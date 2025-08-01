// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "calculate_slowdown_points.hpp"

#include "footprint.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{

lanelet::BasicPolygon2d project_to_pose(
  const autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto rotated_footprint = autoware_utils::rotate_polygon(base_footprint, angle);
  lanelet::BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + pose.position.x, p.y() + pose.position.y);
  return footprint;
}

std::optional<geometry_msgs::msg::Pose> calculate_last_avoiding_pose(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const autoware_utils::Polygon2d & footprint, const lanelet::BasicPolygons2d & polygons_to_avoid,
  const double min_arc_length, const double max_arc_length, const double precision)
{
  geometry_msgs::msg::Pose interpolated_pose{};
  bool is_avoiding_pose = false;

  auto from = min_arc_length;
  auto to = max_arc_length;
  while (to - from > precision) {
    auto l = from + 0.5 * (to - from);
    interpolated_pose = motion_utils::calcInterpolatedPose(trajectory, l);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_pose);
    is_avoiding_pose =
      std::all_of(polygons_to_avoid.begin(), polygons_to_avoid.end(), [&](const auto & polygon) {
        return boost::geometry::disjoint(interpolated_footprint, polygon);
      });
    if (is_avoiding_pose) {
      from = l;
    } else {
      to = l;
    }
  }
  if (is_avoiding_pose) {
    return interpolated_pose;
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Pose> calculate_pose_ahead_of_collision(
  const EgoData & ego_data, const OutOfLanePoint & point_to_avoid,
  const autoware_utils::Polygon2d & footprint, const double precision)
{
  const auto first_avoid_arc_length = motion_utils::calcSignedArcLength(
    ego_data.trajectory_points, 0UL, point_to_avoid.trajectory_index);
  for (auto l = first_avoid_arc_length - precision; l >= ego_data.min_stop_arc_length;
       l -= precision) {
    const auto interpolated_pose =
      motion_utils::calcInterpolatedPose(ego_data.trajectory_points, l);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_pose);
    if (!boost::geometry::disjoint(interpolated_footprint, point_to_avoid.out_overlaps)) {
      return interpolated_pose;
    }
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Pose> calculate_last_in_lane_pose(
  const EgoData & ego_data, const OutOfLanePoint & point_to_avoid, PlannerParam params)
{
  std::optional<geometry_msgs::msg::Pose> last_in_lane_pose;

  const auto outside_idx = point_to_avoid.trajectory_index;
  const auto outside_arc_length =
    motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0UL, outside_idx);

  const auto raw_footprint = make_base_footprint(params, true);  // ignore extra footprint offsets
  const auto base_footprint = make_base_footprint(params);
  params.extra_front_offset += params.lon_dist_buffer;
  params.extra_right_offset += params.lat_dist_buffer;
  params.extra_left_offset += params.lat_dist_buffer;
  const auto expanded_footprint = make_base_footprint(params);  // with added distance buffers
  lanelet::BasicPolygons2d polygons_to_avoid;
  for (const auto & ll : point_to_avoid.overlapped_lanelets) {
    polygons_to_avoid.push_back(ll.polygon2d().basicPolygon());
  }
  // search for the first slowdown decision for which a stop point can be inserted
  // we first try to use the expanded footprint (distance buffers + extra footprint offsets)
  // then we use the base footprint (with distance buffers)
  // finally, we use the raw footprint
  for (const auto & ego_footprint : {expanded_footprint, base_footprint, raw_footprint}) {
    last_in_lane_pose = calculate_last_avoiding_pose(
      ego_data.trajectory_points, ego_footprint, polygons_to_avoid, ego_data.min_stop_arc_length,
      outside_arc_length, params.precision);
    if (last_in_lane_pose) {
      break;
    }
  }
  // fallback to simply stopping ahead of the collision to avoid (regardless of being out of lane or
  // not)
  if (!last_in_lane_pose) {
    last_in_lane_pose = calculate_pose_ahead_of_collision(
      ego_data, point_to_avoid, expanded_footprint, params.precision);
  }
  return last_in_lane_pose;
}

std::optional<geometry_msgs::msg::Pose> calculate_slowdown_pose(
  const EgoData & ego_data, const OutOfLanePoint & out_of_lane_point, const PlannerParam & params)
{
  auto slowdown_pose = calculate_last_in_lane_pose(ego_data, out_of_lane_point, params);
  if (slowdown_pose && params.use_map_stop_lines) {
    // try to use a map stop line ahead of the stop pose
    auto stop_arc_length =
      motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0LU, slowdown_pose->position);
    for (const auto & stop_point : ego_data.map_stop_points) {
      if (
        stop_point.ego_trajectory_arc_length < stop_arc_length &&
        stop_point.ego_trajectory_arc_length >= ego_data.min_stop_arc_length) {
        slowdown_pose = stop_point.ego_stop_pose;
        stop_arc_length = stop_point.ego_trajectory_arc_length;
      }
    }
  }
  return slowdown_pose;
}

void calculate_min_stop_and_slowdown_distances(
  out_of_lane::EgoData & ego_data, const PlannerData & planner_data,
  const std::optional<geometry_msgs::msg::Pose> & previous_slowdown_pose)
{
  ego_data.min_stop_distance = planner_data.calculate_min_deceleration_distance(0.0).value_or(0.0);
  if (previous_slowdown_pose) {
    // Ensure we do not remove the previous slowdown point due to the min distance limit
    const auto previous_slowdown_pose_arc_length = motion_utils::calcSignedArcLength(
      ego_data.trajectory_points, 0UL, previous_slowdown_pose->position);
    ego_data.min_stop_distance =
      std::min(previous_slowdown_pose_arc_length, ego_data.min_stop_distance);
  }
  ego_data.min_stop_arc_length =
    ego_data.longitudinal_offset_to_first_trajectory_index + ego_data.min_stop_distance;
}

}  // namespace autoware::motion_velocity_planner::out_of_lane
