// Copyright 2021 Intelligent Robotics Lab
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

#ifndef BR2_ODVFF_AVOIDANCE__HEADCONTROLLER_HPP_
#define BR2_ODVFF_AVOIDANCE__HEADCONTROLLER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "br2_tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "br2_odvff_avoidance/PIDController.hpp"

#include "image_transport/image_transport.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_odvff_avoidance
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct VFFVectors
{
  std::vector<float> attractive;
  std::vector<float> repulsive;
  std::vector<float> result;
};

typedef enum {RED, GREEN, BLUE, NUM_COLORS} VFFColor;

class HeadController : public rclcpp_lifecycle::LifecycleNode
{
public:
  HeadController();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void control_sycle();

  void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
  void command_callback(br2_tracking_msgs::msg::PanTiltCommand::UniquePtr msg);

protected:
  VFFVectors get_vff();

  visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors & vff_vectors);
  visualization_msgs::msg::Marker make_marker(const std::vector<float> & vector, VFFColor vff_color);

private:
  rclcpp::Subscription<br2_tracking_msgs::msg::PanTiltCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr 
    joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
  
  control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;
  br2_tracking_msgs::msg::PanTiltCommand::UniquePtr last_command_;
  rclcpp::Time last_command_ts_;

  PIDController pan_pid_, tilt_pid_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace br2_odvff_avoidance

#endif  // BR2_ODVFF_AVOIDANCE__HEADCONTROLLER_HPP_
