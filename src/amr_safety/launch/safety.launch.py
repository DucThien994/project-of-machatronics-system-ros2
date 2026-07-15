#!/usr/bin/env python3
"""
safety.launch.py — Launch collision_warning_node

Pipeline (ros2_control):
  Nav2/teleop → /cmd_vel → collision_warning_node → /cmd_vel_safe
                                                         ↓ (remap, xem dưới)
                                     /mecanum_drive_controller/reference_unstamped
                                     → mecanum_drive_controller → 4 khớp bánh (Gazebo)

FIX: trước đây node publish /cmd_vel_safe nhưng KHÔNG remap đi đâu cả sau khi
gỡ planar_move — robot sẽ đứng yên vì không ai subscribe /cmd_vel_safe.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_safety',
            executable='collision_warning_node',
            name='collision_warning_node',
            output='screen',
            parameters=[{
                'publish_rate': 20.0,   # Hz
                'cmd_timeout':   0.5,   # s
            }],
            remappings=[('/cmd_vel_safe',
                         '/mecanum_drive_controller/reference_unstamped')],
        )
    ])
