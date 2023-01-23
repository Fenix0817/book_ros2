// Robot moves exercise.
// Page 79.

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "br2_tf2_detector/RobotMoveNode.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{
  using namespace std::chrono_literals;
  RobotMoveNode::RobotMoveNode()
  : Node("robot_moves"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
  {
    timer_ = create_wall_timer(1s, std::bind(&RobotMoveNode::control_cycle, this));
  }

  void RobotMoveNode::control_cycle()
  {
    geometry_msgs::msg::TransformStamped odom2base_footprint;

    try {
      odom2base_footprint = tf_buffer_.lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
      return;
    }

    double x = odom2base_footprint.transform.translation.x;
    double y = odom2base_footprint.transform.translation.y;
    double z = odom2base_footprint.transform.translation.z;
    double theta = atan2(y, x);

    RCLCPP_INFO(
      get_logger(), "Obstacle detected at (%lf m, %lf m, , %lf m) = %lf rads",
      x, y, z, theta);
  }

}
