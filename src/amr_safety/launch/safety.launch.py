#!/usr/bin/env python3
"""
safety.launch.py — Launch collision_warning_node

Pipeline:
  Nav2/teleop → /cmd_vel → collision_warning_node → /cmd_vel_safe
                                                         ↓
                                             planar_move plugin
                                        (URDF: remapping cmd_vel:=cmd_vel_safe)
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
            # Không remap — publish /cmd_vel_safe trực tiếp
            # planar_move plugin đọc /cmd_vel_safe qua internal remap
        )
    ])
