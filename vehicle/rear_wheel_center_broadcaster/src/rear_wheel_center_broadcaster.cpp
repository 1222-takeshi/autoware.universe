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
        std::bind(&RearWheelCenterBroadcaster::handleChangeWheelCenter, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&RearWheelCenterBroadcaster::broadcastTransform, this));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    wheel_center_x_ = wheel_rear_center_x_;
    wheel_center_yaw_ = 0.0;
}

void RearWheelCenterBroadcaster::broadcastTransform() {
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion tf_q;
    tf_q.setRPY(wheel_center_roll_, wheel_center_pitch_, wheel_center_yaw_);
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "rear_wheel_center";
    t.transform.translation.x = wheel_center_x_;
    t.transform.translation.y = wheel_center_y_;
    t.transform.translation.z = wheel_center_y_;
    t.transform.rotation.x = tf_q.x();
    t.transform.rotation.y = tf_q.y();
    t.transform.rotation.z = tf_q.z();
    t.transform.rotation.w = tf_q.w();

    tf_broadcaster_->sendTransform(t);
}

void RearWheelCenterBroadcaster::handleChangeWheelCenter(const std_msgs::msg::Bool::SharedPtr msg) {
    std_msgs::msg::String state_msg;
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

}  // namespace rear_wheel_center_broadcaster

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rear_wheel_center_broadcaster::RearWheelCenterBroadcaster)
