// Copyright 2022 Tier IV, Inc.
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
#ifndef REAR_WHEEL_CENTER_BROADCASTER__REAR_WHEEL_CENTER_BROADCASTER_HPP_
#define REAR_WHEEL_CENTER_BROADCASTER__REAR_WHEEL_CENTER_BROADCASTER_HPP_

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rear_wheel_center_broadcaster
{
class RearWheelCenterBroadcaster : public rclcpp::Node {
public:
    explicit RearWheelCenterBroadcaster(const rclcpp::NodeOptions & node_options);

private:
    void broadcastTransform();
    void onChangeWheelCenter(const std_msgs::msg::Bool::SharedPtr msg);
    void onKinematicState(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr change_wheel_center_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

    double wheel_center_x_;
    double wheel_center_y_;
    double wheel_center_z_;
    double wheel_center_roll_;
    double wheel_center_pitch_;
    double wheel_center_yaw_;
    double wheel_front_center_x_;
    double wheel_rear_center_x_;
    double change_yaw_;
    bool initialpose_published_;
};

}  // namespace rear_wheel_center_broadcaster
#endif // REAR_WHEEL_CENTER_BROADCASTER__REAR_WHEEL_CENTER_BROADCASTER_HPP_
