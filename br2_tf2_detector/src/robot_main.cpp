// Robot moves exercise.
// Page 79.

#include <memory>

#include "br2_tf2_detector/RobotMoveNode.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto robot_moves = std::make_shared<br2_tf2_detector::RobotMoveNode>();
  rclcpp::spin(robot_moves);

  rclcpp::shutdown();

  return 0;
}
