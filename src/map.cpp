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

#include "adore_map/map.hpp"

#include "adore_map/helpers.hpp"

namespace adore
{
namespace map
{

std::vector<std::shared_ptr<Lane>>
Map::get_neighbour_lanes( size_t lane_id ) const
{
  std::vector<std::shared_ptr<Lane>> neighbour_lanes;

  auto it = lanes.find( lane_id );
  if( it == lanes.end() )
  {
    return neighbour_lanes; // Return empty if lane_id not found
  }

  const auto& target_lane = it->second;
  const auto& target_road = roads.at( target_lane->road_id );

  for( const auto& lane_ptr : target_road.lanes )
  {
    if( lane_ptr->id != lane_id )
    {
      neighbour_lanes.push_back( lane_ptr );
    }
  }

  return neighbour_lanes;
}

double
Map::get_lane_speed_limit( size_t lane_id ) const
{
  // Use find() to search for the lane_id
  auto it = lanes.find( lane_id );
  if( it != lanes.end() )
  {
    const auto& lane        = it->second;
    double      speed_limit = lane->get_speed_limit();
    return speed_limit;
  }

  return 13.6;
}

} // namespace map
} // namespace adore
