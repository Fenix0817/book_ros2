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

#include <algorithm>
#include <utility>

#include "br2_odvff_avoidance/HeadController.hpp"
#include "br2_odvff_avoidance/PIDController.hpp"

#include "br2_tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_odvff_avoidance
{

using std::placeholders::_1;
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

HeadController::HeadController()
: LifecycleNode("head_tracker"),
  pan_pid_(0.0, 1.0, 0.0, 0.3),
  tilt_pid_(0.0, 1.0, 0.0, 0.3),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  command_sub_ = create_subscription<br2_tracking_msgs::msg::PanTiltCommand>(
    "command", 100,
    std::bind(&HeadController::command_callback, this, _1));
  joint_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "joint_state", rclcpp::SensorDataQoS(),
    std::bind(&HeadController::joint_state_callback, this, _1));
  joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_command", 100);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
  vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 100);
}

CallbackReturn
HeadController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(get_logger(), "HeadController configured");

  pan_pid_.set_pid(0.4, 0.05, 0.55);
  tilt_pid_.set_pid(0.4, 0.05, 0.55);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
HeadController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(get_logger(), "HeadController activated");

  joint_pub_->on_activate();
  timer_ = create_wall_timer(100ms, std::bind(&HeadController::control_sycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
HeadController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(get_logger(), "HeadController deactivated");

  try
  {
    trajectory_msgs::msg::JointTrajectory command_msg;
    command_msg.header.stamp = now();
    command_msg.joint_names = last_state_->joint_names;
    command_msg.points.resize(1);
    command_msg.points[0].positions.resize(2);
    command_msg.points[0].velocities.resize(2);
    command_msg.points[0].accelerations.resize(2);
    command_msg.points[0].positions[0] = 0.0;
    command_msg.points[0].positions[1] = 0.0;
    command_msg.points[0].velocities[0] = 0.1;
    command_msg.points[0].velocities[1] = 0.1;
    command_msg.points[0].accelerations[0] = 0.1;
    command_msg.points[0].accelerations[1] = 0.1;
    command_msg.points[0].time_from_start = rclcpp::Duration(1s);

    joint_pub_->publish(command_msg);

    joint_pub_->on_deactivate();
    timer_ = nullptr;
  }
  catch(...)
  {
    std::cout << "Here" << std::endl;
    // return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

void
HeadController::joint_state_callback(
    control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg)
{
  last_state_ = std::move(msg);
}

void
HeadController::command_callback(br2_tracking_msgs::msg::PanTiltCommand::UniquePtr msg)
{
  last_command_ = std::move(msg);
  last_command_ts_ = now();
}

void
HeadController::control_sycle()
{
  if (last_state_ == nullptr)
  {
    return;
  }

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  command_msg.joint_names = last_state_->joint_names;
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].time_from_start = rclcpp::Duration(200ms);

  if (last_command_ == nullptr || (now() - last_command_ts_) > 100ms)
  {
    command_msg.points[0].positions[0] = 0.0;
    command_msg.points[0].positions[1] = 0.0;
    command_msg.points[0].velocities[0] = 0.1;
    command_msg.points[0].velocities[1] = 0.1;
    command_msg.points[0].accelerations[0] = 0.1;
    command_msg.points[0].accelerations[1] = 0.1;
    command_msg.points[0].time_from_start = rclcpp::Duration(1s);
  }
  else
  {
    double control_pan = pan_pid_.get_output(last_command_->pan);
    double control_tilt = tilt_pid_.get_output(last_command_->tilt);

    command_msg.points[0].positions[0] = last_state_->actual.positions[0] - control_pan;
    command_msg.points[0].positions[1] = last_state_->actual.positions[1] - control_tilt;

    command_msg.points[0].velocities[0] = 0.5;
    command_msg.points[0].velocities[1] = 0.5;
    command_msg.points[0].accelerations[0] = 0.5;
    command_msg.points[0].accelerations[1] = 0.5;
  }

  joint_pub_->publish(command_msg);

  // Functions from AvoidanceNode
  const VFFVectors &vff = get_vff();

  // Use result vector to calculate output speed
  const auto & v = vff.result;
  double angle = atan2(v[1], v[0]);
  double module = sqrt(v[0] * v[0] + v[1] * v[1]);

  // Create ouput message, controlling speed limits
  geometry_msgs::msg::Twist vel;
  vel.linear.x = std::clamp(module, 0.0, 0.3);  // truncate linear vel to [0.0, 0.3] m/s
  vel.angular.z = std::clamp(angle, -0.5, 0.5);  // truncate rotation vel to [-0.5, 0.5] rad/s

  vel_pub_->publish(vel);

  // Produce debug information, if any interested
  if (vff_debug_pub_->get_subscription_count() > 0) {
    vff_debug_pub_->publish(get_debug_vff(vff));
  }

}

VFFVectors
HeadController::get_vff()
{
  // This is the obstacle radious in which an obstacle affects the robot
  const float OBSTACLE_DISTANCE = 1.0;

  // Init vectors
  VFFVectors vff_vector;
  vff_vector.attractive = {OBSTACLE_DISTANCE, 0.0};  // Robot wants to go forward
  vff_vector.repulsive = {0.0, 0.0};
  vff_vector.result = {0.0, 0.0};

  geometry_msgs::msg::TransformStamped headAngle;

  headAngle = tf_buffer_.lookupTransform("head_1_link", "base_laser_link", tf2::TimePointZero);

  // Orientation quaternion
  tf2::Quaternion q(
      headAngle.transform.rotation.x,
      headAngle.transform.rotation.y,
      headAngle.transform.rotation.z,
      headAngle.transform.rotation.w);
  
  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);
  
  // Roll Pitch and Yaw from rotation matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  RCLCPP_INFO(get_logger(), " Yaw angle (%lf)", yaw);

  float oposite_angle = yaw + M_PI;
  // The module of the vector is inverse to the distance to the obstacle
  // float complementary_dist = OBSTACLE_DISTANCE - distance_min;
  float complementary_dist = 0.8;

  // Get cartesian (x, y) components from polar (angle, distance)
  vff_vector.repulsive[0] = cos(oposite_angle) * complementary_dist;
  vff_vector.repulsive[1] = sin(oposite_angle) * complementary_dist;

  // Calculate resulting vector adding attractive and repulsive vectors
  vff_vector.result[0] = (vff_vector.repulsive[0] + vff_vector.attractive[0]);
  vff_vector.result[1] = (vff_vector.repulsive[1] + vff_vector.attractive[1]);

  return vff_vector;
}

visualization_msgs::msg::MarkerArray
HeadController::get_debug_vff(const VFFVectors & vff_vectors)
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
  marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
  marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN));

  return marker_array;
}

visualization_msgs::msg::Marker
HeadController::make_marker(const std::vector<float> & vector, VFFColor vff_color)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "base_footprint";
  marker.header.stamp = now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  geometry_msgs::msg::Point end;
  start.x = vector[0];
  start.y = vector[1];
  marker.points = {end, start};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;

  switch (vff_color) {
    case RED:
      marker.id = 0;
      marker.color.r = 1.0;
      break;
    case GREEN:
      marker.id = 1;
      marker.color.g = 1.0;
      break;
    case BLUE:
      marker.id = 2;
      marker.color.b = 1.0;
      break;
  }
  marker.color.a = 1.0;

  return marker;
}

} // namespace br2_odvff_avoidance
