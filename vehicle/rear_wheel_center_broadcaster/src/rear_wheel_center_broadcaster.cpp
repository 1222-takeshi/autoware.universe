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

#include "rear_wheel_center_broadcaster/rear_wheel_center_broadcaster.hpp"

namespace rear_wheel_center_broadcaster
{
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::getRPY;

RearWheelCenterBroadcaster::RearWheelCenterBroadcaster(const rclcpp::NodeOptions & node_options)
: Node("rear_wheel_center_broadcaster", node_options)
{
    wheel_front_center_x_ = declare_parameter("wheel_front_center_x", 0.555);
    wheel_rear_center_x_ = declare_parameter("wheel_rear_center_x", 0.0);
    wheel_center_y_ = declare_parameter("wheel_center_y", 0.0);
    wheel_center_z_ = declare_parameter("wheel_center_z", 0.0);
    wheel_center_roll_ = declare_parameter("wheel_center_roll", 0.0);
    wheel_center_pitch_ = declare_parameter("wheel_center_pitch", 0.0);
    change_yaw_ = declare_parameter("change_yaw", M_PI);

    change_wheel_center_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/change_wheel_center", 10,
        std::bind(&RearWheelCenterBroadcaster::onChangeWheelCenter, this, std::placeholders::_1));
    kinematic_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", 10,
        std::bind(&RearWheelCenterBroadcaster::onKinematicState, this, std::placeholders::_1));

    initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&RearWheelCenterBroadcaster::broadcastTransform, this));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    wheel_center_x_ = wheel_rear_center_x_;
    wheel_center_yaw_ = 0.0;
    initialpose_published_ = false;
}

void RearWheelCenterBroadcaster::broadcastTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "rear_wheel_center";
    t.transform.translation.x = wheel_center_x_;
    t.transform.translation.y = wheel_center_y_;
    t.transform.translation.z = wheel_center_y_;
    t.transform.rotation = createQuaternionFromRPY(wheel_center_roll_, wheel_center_pitch_, wheel_center_yaw_);

    tf_broadcaster_->sendTransform(t);
}

void RearWheelCenterBroadcaster::onChangeWheelCenter(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        wheel_center_x_ = wheel_front_center_x_;
        wheel_center_yaw_ = change_yaw_;
        RCLCPP_INFO(this->get_logger(), "change wheel center to front");
    }else {
        wheel_center_x_ = wheel_rear_center_x_;
        wheel_center_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "change wheel center to rear");
    }
}

void RearWheelCenterBroadcaster::onKinematicState(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped initialpose_msg;
    // if(!initialpose_published_) {
    //     const auto rpy = getRPY(msg->pose.pose.orientation);
    //     initialpose_msg.header.stamp = this->now();
    //     initialpose_msg.header.frame_id = "map";
    //     initialpose_msg.pose.pose.position = msg->pose.pose.position;
    //     initialpose_msg.pose.pose.orientation = createQuaternionFromRPY(rpy.x, rpy.y, wheel_center_yaw_);
    //     initialpose_msg.pose.covariance = msg->pose.covariance;
    //     initialpose_pub_->publish(initialpose_msg);
    //     initialpose_published_ = true;
    //     RCLCPP_INFO(this->get_logger(), "publish initialpose");
    // }
}
}  // namespace rear_wheel_center_broadcaster

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rear_wheel_center_broadcaster::RearWheelCenterBroadcaster)
