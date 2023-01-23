// Robot moves exercise.
// Page 79.

#ifndef BR2_TF2_DETECTOR__ROBOTMOVESNODE_HPP_
#define BR2_TF2_DETECTOR__ROBOTMOVESNODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

class RobotMoveNode : public rclcpp:: Node
{
public:
  RobotMoveNode();

private:
  void control_cycle();
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace br2_tf2_detector
#endif  // BR2_TF2_DETECTOR__ROBOTMOVESNODE_HPP_
