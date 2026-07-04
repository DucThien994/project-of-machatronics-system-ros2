#!/usr/bin/env python3
# teleop.launch.py — ver5.0
# Publish /cmd_vel → collision_warning_node → /cmd_vel_safe → robot
# Cách chạy tốt nhất: ros2 run teleop_twist_keyboard teleop_twist_keyboard
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')],
        )
    ])
