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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{
using Polygons = boost::geometry::model::multi_polygon<lanelet::BasicPolygonWithHoles2d>;

/// @brief parameters for the out_of_lane module
struct PlannerParam
{
  std::string mode;                  // mode used to consider a conflict with an object
  bool skip_if_already_overlapping;  // if true, do not run the module when ego already overlaps
                                     // another lane
  double max_arc_length;  // [m] maximum arc length along the trajectory to check for collision

  double time_threshold;  // [s](mode="threshold") objects time threshold
  double ttc_threshold;  // [s](mode="ttc") threshold on time to collision between ego and an object
  double ttc_release_threshold;

  bool objects_cut_predicted_paths_beyond_red_lights;  // whether to cut predicted paths beyond red
                                                       // lights' stop lines
  double objects_min_vel;          // [m/s] objects lower than this velocity will be ignored
  double objects_min_confidence;   // minimum confidence to consider a predicted path
  bool objects_ignore_behind_ego;  // if true, objects behind the ego vehicle are ignored
  bool
    validate_predicted_paths_on_lanelets;  // if true, an out of lane collision is only considered
                                           // if the predicted path fully follows a sequence of
                                           // lanelets that include the out of lane lanelet
  double objects_extra_width;              // [m] extra width to apply to the object footprints

  // action to insert in the trajectory if an object causes a collision at an overlap
  double lon_dist_buffer;      // [m] safety distance buffer to keep in front of the ego vehicle
  double lat_dist_buffer;      // [m] safety distance buffer to keep on the side of the ego vehicle
  double slow_velocity;        // [m/s] slowdown velocity
  double stop_dist_threshold;  // [m] if a collision is detected bellow this distance ahead of ego,
                               // try to insert a stop point
  double precision;            // [m] precision when inserting a stop pose in the trajectory
  double min_on_duration;   // [s] duration needed before a stop or slowdown point can be triggered
  double min_off_duration;  // [s] duration needed before a stop or slowdown point can be removed
  double update_distance_th;  // [m] distance threshold for updating previous stop pose position
  bool use_map_stop_lines;    // if true, try to stop at stop lines defined in the map

  // ego dimensions used to create its polygon footprint
  double front_offset;        // [m]  front offset (from vehicle info)
  double rear_offset;         // [m]  rear offset (from vehicle info)
  double right_offset;        // [m]  right offset (from vehicle info)
  double left_offset;         // [m]  left offset (from vehicle info)
  double extra_front_offset;  // [m] extra front distance
  double extra_rear_offset;   // [m] extra rear distance
  double extra_right_offset;  // [m] extra right distance
  double extra_left_offset;   // [m] extra left distance
};

namespace bgi = boost::geometry::index;
struct StopLine
{
  autoware_utils::LineString2d stop_line;
  lanelet::ConstLanelets lanelets;
};
using StopLineNode = std::pair<autoware_utils::Box2d, StopLine>;
using StopLinesRtree = bgi::rtree<StopLineNode, bgi::rstar<16>>;
using OutAreaNode = std::pair<autoware_utils::Box2d, size_t>;
using OutAreaRtree = bgi::rtree<OutAreaNode, bgi::rstar<16>>;
using LaneletNode = std::pair<autoware_utils::Box2d, size_t>;
using OutLaneletRtree = bgi::rtree<LaneletNode, bgi::rstar<16>>;

/// @brief data related to the ego vehicle
struct EgoData
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint>
    trajectory_points;  // filtered trajectory starting from the 1st point behind ego
  geometry_msgs::msg::Pose pose;
  double velocity{};              // [m/s] current longitudinal velocity of the ego vehicle
  size_t first_trajectory_idx{};  // segment index closest to ego on the original trajectory
  double
    longitudinal_offset_to_first_trajectory_index{};  // longitudinal offset of ego along the
                                                      // closest segment on the original trajectory
  double min_stop_distance{};
  double min_stop_arc_length{};  // [m] minimum arc length along the filtered trajectory where ego
                                 // can stop

  lanelet::ConstLanelets out_lanelets;  // lanelets where ego would be considered "out of lane"
  OutLaneletRtree out_lanelets_rtree;

  lanelet::BasicPolygon2d current_footprint;
  std::vector<lanelet::BasicPolygon2d>
    trajectory_footprints;  // ego footprints along the filtered trajectory

  StopLinesRtree stop_lines_rtree;  // rtree with the stop lines for other vehicles
  std::vector<StopPoint>
    map_stop_points;  // ego stop points (and their corresponding stop lines) taken from the map
};

/// @brief a collision time along with the object and path id that cause the collision
struct CollisionTime
{
  double collision_time{};
  unique_identifier_msgs::msg::UUID object_uuid;
  size_t object_path_id{};

  // Overload needed to store the struct in std::set
  bool operator<(const CollisionTime & other) const
  {
    return collision_time < other.collision_time;
  }
};

/// @brief data related to an out of lane trajectory point
struct OutOfLanePoint
{
  size_t trajectory_index;
  autoware_utils::MultiPolygon2d out_overlaps;
  std::set<CollisionTime> collision_times;
  std::optional<double> min_object_arrival_time;
  std::optional<double> max_object_arrival_time;
  std::optional<double> ttc;
  lanelet::ConstLanelets overlapped_lanelets;
  bool to_avoid = false;
};

struct SlowdownPose
{
  double arc_length{0.0};
  rclcpp::Time start_time{0};
  geometry_msgs::msg::Pose pose;
  bool is_active = false;

  SlowdownPose() = default;
  SlowdownPose(
    const double arc_length, const rclcpp::Time & start_time, const geometry_msgs::msg::Pose & pose,
    const bool is_active)
  : arc_length(arc_length), start_time(start_time), pose(pose), is_active(is_active)
  {
  }
};

/// @brief data related to the out of lane points
struct OutOfLaneData
{
  std::vector<OutOfLanePoint> outside_points;
  OutAreaRtree outside_areas_rtree;
};

/// @brief debug data
struct DebugData
{
  size_t prev_out_of_lane_areas = 0;
  size_t prev_ttcs = 0;
  size_t prev_objects = 0;
  size_t prev_stop_line = 0;
};

}  // namespace autoware::motion_velocity_planner::out_of_lane

#endif  // TYPES_HPP_
