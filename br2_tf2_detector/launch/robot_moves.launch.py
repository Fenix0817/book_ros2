# Robot moves exercise.
# Page 79.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    detector_cmd = Node(package='br2_tf2_detector',
                        executable='robot_moves',
                        output='screen',
                        parameters=[{
                            'use_sim_time': True
                        }])

    ld = LaunchDescription()
    ld.add_action(detector_cmd)

    return ld
