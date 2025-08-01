// Copyright 2020 Tier IV, Inc.
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

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;

IntersectionModuleManager::IntersectionModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc.intersection")),
  occlusion_rtc_interface_(
    &node, "intersection_occlusion",
    getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc.intersection_to_occlusion"))
{
  const std::string ns(IntersectionModuleManager::getModuleName());
  auto & ip = intersection_param_;

  // common
  {
    ip.common.attention_area_length =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_length");
    ip.common.attention_area_margin =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_margin");
    ip.common.attention_area_angle_threshold =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_angle_threshold");
    ip.common.use_intersection_area =
      get_or_declare_parameter<bool>(node, ns + ".common.use_intersection_area");
    ip.common.default_stopline_margin =
      get_or_declare_parameter<double>(node, ns + ".common.default_stopline_margin");
    ip.common.stopline_overshoot_margin =
      get_or_declare_parameter<double>(node, ns + ".common.stopline_overshoot_margin");
    ip.common.path_interpolation_ds =
      get_or_declare_parameter<double>(node, ns + ".common.path_interpolation_ds");
    ip.common.max_accel = get_or_declare_parameter<double>(node, ns + ".common.max_accel");
    ip.common.max_jerk = get_or_declare_parameter<double>(node, ns + ".common.max_jerk");
    ip.common.delay_response_time =
      get_or_declare_parameter<double>(node, ns + ".common.delay_response_time");
    ip.common.enable_pass_judge_before_default_stopline = get_or_declare_parameter<bool>(
      node, ns + ".common.enable_pass_judge_before_default_stopline");
  }

  // stuck
  {
    // target_type
    {
      ip.stuck_vehicle.target_type.car =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.car");
      ip.stuck_vehicle.target_type.bus =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.bus");
      ip.stuck_vehicle.target_type.truck =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.truck");
      ip.stuck_vehicle.target_type.trailer =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.trailer");
      ip.stuck_vehicle.target_type.motorcycle =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.motorcycle");
      ip.stuck_vehicle.target_type.bicycle =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.bicycle");
      ip.stuck_vehicle.target_type.unknown =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.target_type.unknown");
    }

    // turn_direction
    {
      ip.stuck_vehicle.turn_direction.left =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.turn_direction.left");
      ip.stuck_vehicle.turn_direction.right =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.turn_direction.right");
      ip.stuck_vehicle.turn_direction.straight =
        get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.turn_direction.straight");
    }

    ip.stuck_vehicle.use_stuck_stopline =
      get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.use_stuck_stopline");
    ip.stuck_vehicle.stuck_vehicle_detect_dist =
      get_or_declare_parameter<double>(node, ns + ".stuck_vehicle.stuck_vehicle_detect_dist");
    ip.stuck_vehicle.stuck_vehicle_velocity_threshold = get_or_declare_parameter<double>(
      node, ns + ".stuck_vehicle.stuck_vehicle_velocity_threshold");
    ip.stuck_vehicle.disable_against_private_lane =
      get_or_declare_parameter<bool>(node, ns + ".stuck_vehicle.disable_against_private_lane");
  }

  // yield_stuck
  {
    // target_type
    {
      ip.yield_stuck.target_type.car =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.car");
      ip.yield_stuck.target_type.bus =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.bus");
      ip.yield_stuck.target_type.truck =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.truck");
      ip.yield_stuck.target_type.trailer =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.trailer");
      ip.yield_stuck.target_type.motorcycle =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.motorcycle");
      ip.yield_stuck.target_type.bicycle =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.bicycle");
      ip.yield_stuck.target_type.unknown =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.target_type.unknown");
    }

    // turn_direction
    {
      ip.yield_stuck.turn_direction.left =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.turn_direction.left");
      ip.yield_stuck.turn_direction.right =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.turn_direction.right");
      ip.yield_stuck.turn_direction.straight =
        get_or_declare_parameter<bool>(node, ns + ".yield_stuck.turn_direction.straight");
    }

    ip.yield_stuck.distance_threshold =
      get_or_declare_parameter<double>(node, ns + ".yield_stuck.distance_threshold");
  }

  // collision_detection
  {
    ip.collision_detection.consider_wrong_direction_vehicle = get_or_declare_parameter<bool>(
      node, ns + ".collision_detection.consider_wrong_direction_vehicle");
    ip.collision_detection.collision_detection_hold_time = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.collision_detection_hold_time");
    ip.collision_detection.min_predicted_path_confidence = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.min_predicted_path_confidence");

    // target_type
    {
      ip.collision_detection.target_type.car =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.car");
      ip.collision_detection.target_type.bus =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.bus");
      ip.collision_detection.target_type.truck =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.truck");
      ip.collision_detection.target_type.trailer =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.trailer");
      ip.collision_detection.target_type.motorcycle =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.motorcycle");
      ip.collision_detection.target_type.bicycle =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.bicycle");
      ip.collision_detection.target_type.unknown =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.unknown");
    }

    // velocity_profile
    {
      ip.collision_detection.velocity_profile.use_upstream = get_or_declare_parameter<bool>(
        node, ns + ".collision_detection.velocity_profile.use_upstream");
      ip.collision_detection.velocity_profile.minimum_upstream_velocity =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.velocity_profile.minimum_upstream_velocity");
      ip.collision_detection.velocity_profile.default_velocity = get_or_declare_parameter<double>(
        node, ns + ".collision_detection.velocity_profile.default_velocity");
      ip.collision_detection.velocity_profile.minimum_default_velocity =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.velocity_profile.minimum_default_velocity");
    }

    // fully_prioritized
    {
      ip.collision_detection.fully_prioritized.collision_start_margin_time =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.fully_prioritized.collision_start_margin_time");
      ip.collision_detection.fully_prioritized.collision_end_margin_time =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.fully_prioritized.collision_end_margin_time");
    }

    // partially_prioritized
    {
      ip.collision_detection.partially_prioritized.collision_start_margin_time =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.partially_prioritized.collision_start_margin_time");
      ip.collision_detection.partially_prioritized.collision_end_margin_time =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.partially_prioritized.collision_end_margin_time");
    }

    // not_prioritized
    {
      ip.collision_detection.not_prioritized.collision_start_margin_time =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.not_prioritized.collision_start_margin_time");
      ip.collision_detection.not_prioritized.collision_end_margin_time =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.not_prioritized.collision_end_margin_time");
    }

    // yield_on_green_traffic_light
    {
      ip.collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start =
        get_or_declare_parameter<double>(
          node,
          ns +
            ".collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start");
      ip.collision_detection.yield_on_green_traffic_light.duration =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.yield_on_green_traffic_light.duration");
      ip.collision_detection.yield_on_green_traffic_light.object_dist_to_stopline =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.yield_on_green_traffic_light.object_dist_to_stopline");
    }

    // ignore_on_amber_traffic_light, ignore_on_red_traffic_light
    {
      ip.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.car =
        get_or_declare_parameter<double>(
          node,
          ns +
            ".collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.car");
      ip.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.bike =
        get_or_declare_parameter<double>(
          node,
          ns +
            ".collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.bike");
      ip.collision_detection.ignore_on_red_traffic_light.object_margin_to_path =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.ignore_on_red_traffic_light.object_margin_to_path");
    }

    ip.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point =
      get_or_declare_parameter<double>(
        node, ns +
                ".collision_detection.avoid_collision_by_acceleration.object_time_margin_to_"
                "collision_point");
  }

  // occlusion
  {
    ip.occlusion.enable = get_or_declare_parameter<bool>(node, ns + ".occlusion.enable");
    ip.occlusion.request_approval_wo_traffic_light =
      get_or_declare_parameter<bool>(node, ns + ".occlusion.request_approval_wo_traffic_light");
    ip.occlusion.occlusion_attention_area_length =
      get_or_declare_parameter<double>(node, ns + ".occlusion.occlusion_attention_area_length");
    ip.occlusion.free_space_max =
      get_or_declare_parameter<int>(node, ns + ".occlusion.free_space_max");
    ip.occlusion.occupied_min = get_or_declare_parameter<int>(node, ns + ".occlusion.occupied_min");
    ip.occlusion.denoise_kernel =
      get_or_declare_parameter<double>(node, ns + ".occlusion.denoise_kernel");
    ip.occlusion.attention_lane_crop_curvature_threshold = get_or_declare_parameter<double>(
      node, ns + ".occlusion.attention_lane_crop_curvature_threshold");
    ip.occlusion.attention_lane_curvature_calculation_ds = get_or_declare_parameter<double>(
      node, ns + ".occlusion.attention_lane_curvature_calculation_ds");

    // creep_during_peeking
    {
      ip.occlusion.creep_during_peeking.enable =
        get_or_declare_parameter<bool>(node, ns + ".occlusion.creep_during_peeking.enable");
      ip.occlusion.creep_during_peeking.creep_velocity = get_or_declare_parameter<double>(
        node, ns + ".occlusion.creep_during_peeking.creep_velocity");
    }

    ip.occlusion.peeking_offset =
      get_or_declare_parameter<double>(node, ns + ".occlusion.peeking_offset");
    ip.occlusion.occlusion_required_clearance_distance = get_or_declare_parameter<double>(
      node, ns + ".occlusion.occlusion_required_clearance_distance");
    ip.occlusion.possible_object_bbox =
      get_or_declare_parameter<std::vector<double>>(node, ns + ".occlusion.possible_object_bbox");
    ip.occlusion.ignore_parked_vehicle_speed_threshold = get_or_declare_parameter<double>(
      node, ns + ".occlusion.ignore_parked_vehicle_speed_threshold");
    ip.occlusion.occlusion_detection_hold_time =
      get_or_declare_parameter<double>(node, ns + ".occlusion.occlusion_detection_hold_time");
    ip.occlusion.temporal_stop_time_before_peeking =
      get_or_declare_parameter<double>(node, ns + ".occlusion.temporal_stop_time_before_peeking");
    ip.occlusion.creep_velocity_without_traffic_light = get_or_declare_parameter<double>(
      node, ns + ".occlusion.creep_velocity_without_traffic_light");
    ip.occlusion.static_occlusion_with_traffic_light_timeout = get_or_declare_parameter<double>(
      node, ns + ".occlusion.static_occlusion_with_traffic_light_timeout");
  }

  {
    ip.conservative_merging.enable_yield =
      get_or_declare_parameter<bool>(node, ns + ".conservative_merging.enable_yield");
    ip.conservative_merging.minimum_lateral_distance_threshold = get_or_declare_parameter<double>(
      node, ns + ".conservative_merging.minimum_lateral_distance_threshold");
    ip.conservative_merging.merging_judge_angle_threshold = get_or_declare_parameter<double>(
      node, ns + ".conservative_merging.merging_judge_angle_threshold");
  }

  ip.debug.ttc = get_or_declare_parameter<std::vector<int64_t>>(node, ns + ".debug.ttc");

  decision_state_pub_ =
    node.create_publisher<std_msgs::msg::String>("~/debug/intersection/decision_state", 1);
  tl_observation_pub_ = node.create_publisher<autoware_perception_msgs::msg::TrafficLightGroup>(
    "~/debug/intersection_traffic_signal", 1);

  const bool enable_console_output =
    get_or_declare_parameter<bool>(node, "planning_factor_console_output.enable");
  const int throttle_duration_ms =
    get_or_declare_parameter<int>(node, "planning_factor_console_output.duration");

  planning_factor_interface_for_occlusion_ =
    std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      &node, "intersection_occlusion", enable_console_output, throttle_duration_ms);
}

void IntersectionModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);
  // run occlusion detection only in the first intersection
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    // Is intersection?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    const auto is_intersection =
      turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
    if (!is_intersection) {
      continue;
    }

    if (hasSameParentLaneletAndTurnDirectionWithRegistered(ll)) {
      continue;
    }

    const auto associative_ids =
      planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
    bool has_traffic_light = false;
    if (const auto tl_reg_elems = ll.regulatoryElementsAs<lanelet::TrafficLight>();
        tl_reg_elems.size() != 0) {
      const auto tl_reg_elem = tl_reg_elems.front();
      const auto stopline_opt = tl_reg_elem->stopLine();
      if (!!stopline_opt) has_traffic_light = true;
    }
    const auto new_module = std::make_shared<IntersectionModule>(
      module_id, lane_id, planner_data_, intersection_param_, associative_ids, turn_direction,
      has_traffic_light, node_, logger_.get_child("intersection_module"), clock_, time_keeper_,
      planning_factor_interface_, planning_factor_interface_for_occlusion_);
    generate_uuid(module_id);
    /* set RTC status as non_occluded status initially */
    const UUID uuid = getUUID(new_module->getModuleId());
    const auto occlusion_uuid = new_module->getOcclusionUUID();
    rtc_interface_.updateCooperateStatus(
      uuid, true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(), clock_->now());
    occlusion_rtc_interface_.updateCooperateStatus(
      occlusion_uuid, true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(), clock_->now());
    registerModule(std::move(new_module));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
IntersectionModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_set = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lane_set](const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto & associative_ids = intersection_module->getAssociativeIds();
    for (const auto & lane : lane_set) {
      const std::string turn_direction = lane.attributeOr("turn_direction", "else");
      const auto is_intersection =
        turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
      if (!is_intersection) {
        continue;
      }

      if (associative_ids.find(lane.id()) != associative_ids.end() /* contains */) {
        return false;
      }
    }
    return true;
  };
}

bool IntersectionModuleManager::hasSameParentLaneletAndTurnDirectionWithRegistered(
  const lanelet::ConstLanelet & lane) const
{
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto & associative_ids = intersection_module->getAssociativeIds();
    if (associative_ids.find(lane.id()) != associative_ids.end()) {
      return true;
    }
  }
  return false;
}

void IntersectionModuleManager::sendRTC(const Time & stamp)
{
  double min_distance = std::numeric_limits<double>::infinity();
  std::optional<TrafficSignalStamped> nearest_tl_observation{std::nullopt};
  std_msgs::msg::String decision_type;

  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const UUID uuid = getUUID(scene_module->getModuleId());
    const bool safety =
      scene_module->isSafe() && (!intersection_module->isOcclusionFirstStopRequired());
    updateRTCStatus(uuid, safety, State::RUNNING, scene_module->getDistance(), stamp);
    const auto occlusion_uuid = intersection_module->getOcclusionUUID();
    const auto occlusion_distance = intersection_module->getOcclusionDistance();
    const auto occlusion_safety = intersection_module->getOcclusionSafety();
    occlusion_rtc_interface_.updateCooperateStatus(
      occlusion_uuid, occlusion_safety, State::RUNNING, occlusion_distance, occlusion_distance,
      stamp);

    // ==========================================================================================
    // module debug data
    // ==========================================================================================
    const auto internal_debug_data = intersection_module->getInternalDebugData();
    if (internal_debug_data.distance < min_distance) {
      min_distance = internal_debug_data.distance;
      nearest_tl_observation = internal_debug_data.tl_observation;
    }
    decision_type.data += (internal_debug_data.decision_type + "\n");
  }
  rtc_interface_.publishCooperateStatus(stamp);  // publishRTCStatus()
  occlusion_rtc_interface_.publishCooperateStatus(stamp);

  // ==========================================================================================
  // publish module debug data
  // ==========================================================================================
  decision_state_pub_->publish(decision_type);
  if (nearest_tl_observation) {
    tl_observation_pub_->publish(nearest_tl_observation.value().signal);
  }
}

void IntersectionModuleManager::modifyPathVelocity(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path)
{
  SceneModuleManagerInterfaceWithRTC::modifyPathVelocity(path);
  planning_factor_interface_for_occlusion_->publish();
}

void IntersectionModuleManager::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto occlusion_uuid = intersection_module->getOcclusionUUID();
    scene_module->setActivation(rtc_interface_.isActivated(getUUID(scene_module->getModuleId())));
    intersection_module->setOcclusionActivation(
      occlusion_rtc_interface_.isActivated(occlusion_uuid));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(getUUID(scene_module->getModuleId())));
  }
}

void IntersectionModuleManager::deleteExpiredModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      // default
      removeRTCStatus(getUUID((*itr)->getModuleId()));
      removeUUID((*itr)->getModuleId());
      // occlusion
      const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(*itr);
      const auto occlusion_uuid = intersection_module->getOcclusionUUID();
      occlusion_rtc_interface_.removeCooperateStatus(occlusion_uuid);
      registered_module_id_set_.erase((*itr)->getModuleId());
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

MergeFromPrivateModuleManager::MergeFromPrivateModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(MergeFromPrivateModuleManager::getModuleName());
  auto & mp = merge_from_private_area_param_;
  mp.stop_duration_sec = get_or_declare_parameter<double>(node, ns + ".stop_duration_sec");
  mp.attention_area_length =
    node.get_parameter("intersection.common.attention_area_length").as_double();
  mp.stopline_margin = get_or_declare_parameter<double>(node, ns + ".stopline_margin");
  mp.path_interpolation_ds =
    node.get_parameter("intersection.common.path_interpolation_ds").as_double();
  mp.stop_distance_threshold =
    get_or_declare_parameter<double>(node, ns + ".stop_distance_threshold");
}

void MergeFromPrivateModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is intersection?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    const auto is_intersection =
      turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
    if (!is_intersection) {
      continue;
    }

    // Is merging from private road?
    // In case the goal is in private road, check if this lanelet is conflicting with urban lanelet
    const std::string lane_location = ll.attributeOr("location", "else");
    if (lane_location != "private") {
      continue;
    }

    if (hasSameParentLaneletAndTurnDirectionWithRegistered(ll)) {
      continue;
    }

    if (i + 1 < lanelets.size()) {
      const auto next_lane = lanelets.at(i + 1);
      const std::string next_lane_location = next_lane.attributeOr("location", "else");
      if (next_lane_location != "private") {
        const auto associative_ids =
          planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
        registerModule(
          std::make_shared<MergeFromPrivateRoadModule>(
            module_id, lane_id, planner_data_, merge_from_private_area_param_, associative_ids,
            logger_.get_child("merge_from_private_road_module"), clock_, time_keeper_,
            planning_factor_interface_));
        continue;
      }
    } else {
      const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
      const auto conflicting_lanelets =
        lanelet::utils::getConflictingLanelets(routing_graph_ptr, ll);
      for (auto && conflicting_lanelet : conflicting_lanelets) {
        const std::string conflicting_attr = conflicting_lanelet.attributeOr("location", "else");
        if (conflicting_attr == "urban") {
          const auto associative_ids =
            planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
          registerModule(
            std::make_shared<MergeFromPrivateRoadModule>(
              module_id, lane_id, planner_data_, merge_from_private_area_param_, associative_ids,
              logger_.get_child("merge_from_private_road_module"), clock_, time_keeper_,
              planning_factor_interface_));
          continue;
        }
      }
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
MergeFromPrivateModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_set = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lane_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    const auto merge_from_private_module =
      std::dynamic_pointer_cast<MergeFromPrivateRoadModule>(scene_module);
    const auto & associative_ids = merge_from_private_module->getAssociativeIds();
    for (const auto & lane : lane_set) {
      const std::string turn_direction = lane.attributeOr("turn_direction", "else");
      const auto is_intersection =
        turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
      if (!is_intersection) {
        continue;
      }

      if (associative_ids.find(lane.id()) != associative_ids.end() /* contains */) {
        return false;
      }
    }
    return true;
  };
}

bool MergeFromPrivateModuleManager::hasSameParentLaneletAndTurnDirectionWithRegistered(
  const lanelet::ConstLanelet & lane) const
{
  for (const auto & scene_module : scene_modules_) {
    const auto merge_from_private_module =
      std::dynamic_pointer_cast<MergeFromPrivateRoadModule>(scene_module);
    const auto & associative_ids = merge_from_private_module->getAssociativeIds();
    if (associative_ids.find(lane.id()) != associative_ids.end()) {
      return true;
    }
  }
  return false;
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::IntersectionModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::MergeFromPrivateModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
