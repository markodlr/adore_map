/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once
#include <cmath>

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>

#include "adore_math/distance.h"

template<typename Point>
class Quadtree
{
public:

  Quadtree() {};

  // Boundary for this node (a simple square region)
  struct Boundary
  {
    double x_min, x_max, y_min, y_max;

    // Check if a point lies within this boundary
    template<typename QueryPoint>
    bool
    contains( const QueryPoint& point ) const
    {
      return ( point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max );
    }

    // Check if this boundary overlaps with another rectangular boundary
    bool
    intersects( const Boundary& range ) const
    {
      return !( range.x_min > x_max || range.x_max < x_min || range.y_min > y_max || range.y_max < y_min );
    }

    // Squared distance from a point to the rectangle (no sqrt)
    template<typename QueryPoint>
    double
    squared_distance_to_point( const QueryPoint& point ) const
    {
      double dx = 0.0;
      if( point.x < x_min )
        dx = x_min - point.x;
      else if( point.x > x_max )
        dx = point.x - x_max;

      double dy = 0.0;
      if( point.y < y_min )
        dy = y_min - point.y;
      else if( point.y > y_max )
        dy = point.y - y_max;

      return dx * dx + dy * dy;
    }

    // Keep the old API if something else uses it
    template<typename QueryPoint>
    double
    distance_to_point( const QueryPoint& point ) const
    {
      return std::sqrt( squared_distance_to_point( point ) );
    }

    bool
    intersects_circle( double center_x, double center_y, double radius ) const
    {
      double closest_x = std::clamp( center_x, x_min, x_max );
      double closest_y = std::clamp( center_y, y_min, y_max );
      double dx        = closest_x - center_x;
      double dy        = closest_y - center_y;
      return dx * dx + dy * dy <= radius * radius;
    }
  };

  // Constructor for Quadtree node
  Quadtree( const Boundary& boundary, size_t capacity ) :
    boundary( boundary ),
    capacity( capacity ),
    divided( false )
  {}

  // Insert a point into the quadtree
  bool
  insert( const Point& point )
  {
    if( !boundary.contains( point ) )
    {
      return false; // Point is out of this node's boundary
    }

    if( points.size() < capacity )
    {
      points.push_back( point );
      return true;
    }

    // Need to subdivide and redistribute points
    if( !divided )
    {
      subdivide();
    }

    // Now insert the new point into appropriate child
    return ( northwest->insert( point ) || northeast->insert( point ) || southwest->insert( point ) || southeast->insert( point ) );
  }

  // Query all points within a range
  void
  query( const Boundary& range, std::vector<Point>& found ) const
  {
    if( !boundary.intersects( range ) )
    {
      return; // Range does not intersect this node, return
    }

    for( const auto& point : points )
    {
      if( range.contains( point ) )
      {
        found.push_back( point );
      }
    }

    // Recursively check children if divided
    if( divided )
    {
      northwest->query( range, found );
      northeast->query( range, found );
      southwest->query( range, found );
      southeast->query( range, found );
    }
  }

  // Query all points within a given radius from a center point (circular range)
  template<typename QueryPoint>
  void
  query_range( const QueryPoint& center, double radius, std::vector<Point>& found ) const
  {
    // If the query circle does not intersect this node's boundary, return early
    if( !boundary.intersects_circle( center.x, center.y, radius ) )
    {
      return;
    }

    // Check all points in this node
    for( const auto& point : points )
    {
      double distance = std::hypot( point.x - center.x, point.y - center.y );
      if( distance <= radius )
      {
        found.push_back( point );
      }
    }

    // Recursively check children if divided
    if( divided )
    {
      northwest->query_range( center, radius, found );
      northeast->query_range( center, radius, found );
      southwest->query_range( center, radius, found );
      southeast->query_range( center, radius, found );
    }
  }

  // Internal implementation: works with squared distances only.
  template<typename QueryPoint>
  std::optional<Point>
  get_nearest_point_impl( const QueryPoint&                          query_point,
                          double&                                    min_dist2, // squared distance
                          const std::function<bool( const Point& )>& filter ) const
  {
    std::optional<Point> nearest_point = std::nullopt;

    // 1) Check all points in this node
    for( const auto& point : points )
    {
      if( !filter( point ) )
        continue;

      const double d2 = adore::math::squared_distance_2d( point, query_point );
      if( d2 < min_dist2 )
      {
        min_dist2     = d2;
        nearest_point = point;
      }
    }

    // 2) Recursively check children if subdivided
    if( divided )
    {
      struct ChildInfo
      {
        double                 dist2;
        const Quadtree<Point>* node;
      };

      ChildInfo children[4] = {
        { northwest->boundary.squared_distance_to_point( query_point ), northwest.get() },
        { northeast->boundary.squared_distance_to_point( query_point ), northeast.get() },
        { southwest->boundary.squared_distance_to_point( query_point ), southwest.get() },
        { southeast->boundary.squared_distance_to_point( query_point ), southeast.get() }
      };

      std::sort( std::begin( children ), std::end( children ), []( const ChildInfo& a, const ChildInfo& b ) { return a.dist2 < b.dist2; } );

      for( const auto& child : children )
      {
        if( child.dist2 >= min_dist2 )
        {
          // This child cannot contain a closer point than current best
          break;
        }

        if( auto child_nearest = child.node->get_nearest_point_impl( query_point, min_dist2, filter ) )
        {
          nearest_point = child_nearest;
        }
      }
    }

    return nearest_point;
  }

  // Public API: same signature as before, works in true distance.
  template<typename QueryPoint>
  std::optional<Point>
  get_nearest_point(
    const QueryPoint&                          query_point,
    double&                                    min_dist, // true distance (in/out)
    const std::function<bool( const Point& )>& filter = []( const Point& ) { return true; } ) const
  {
    double min_dist2 = min_dist * min_dist;

    auto nearest = get_nearest_point_impl( query_point, min_dist2, filter );

    // If we never improved min_dist2, min_dist stays as given.
    if( min_dist2 < min_dist * min_dist )
    {
      min_dist = std::sqrt( min_dist2 );
    }

    return nearest;
  }

  Boundary boundary;
  size_t   capacity = 10;

private:


  std::vector<Point> points;
  bool               divided = false;

  // Children of the quadtree
  std::shared_ptr<Quadtree<Point>> northwest = nullptr;
  std::shared_ptr<Quadtree<Point>> northeast = nullptr;
  std::shared_ptr<Quadtree<Point>> southwest = nullptr;
  std::shared_ptr<Quadtree<Point>> southeast = nullptr;

  // Subdivide the current node into four smaller nodes
  void
  subdivide()
  {
    double x_mid = ( boundary.x_min + boundary.x_max ) / 2;
    double y_mid = ( boundary.y_min + boundary.y_max ) / 2;

    // Create the four child quadrants
    northwest = std::make_shared<Quadtree<Point>>( Boundary{ boundary.x_min, x_mid, y_mid, boundary.y_max }, capacity );
    northeast = std::make_shared<Quadtree<Point>>( Boundary{ x_mid, boundary.x_max, y_mid, boundary.y_max }, capacity );
    southwest = std::make_shared<Quadtree<Point>>( Boundary{ boundary.x_min, x_mid, boundary.y_min, y_mid }, capacity );
    southeast = std::make_shared<Quadtree<Point>>( Boundary{ x_mid, boundary.x_max, boundary.y_min, y_mid }, capacity );

    divided = true;

    // Redistribute points into children
    for( const auto& p : points )
    {
      // Insert point into appropriate child node
      bool inserted = ( northwest->insert( p ) || northeast->insert( p ) || southwest->insert( p ) || southeast->insert( p ) );
      if( !inserted )
      {
        std::cerr << "subdivision problems - point not in any of sub quads" << std::endl;
        // Handle error: point should have been inserted into a child
        // This should not happen if boundaries are correctly defined
      }
    }
    points.clear(); // Clear points from the parent node
  }
};
