#!/usr/bin/env python3
"""
bringup.launch.py — ver5.0 top-level orchestration
Timeline:
  t=0s    → Gazebo + RSP + spawn robot (gazebo.launch.py)
  t=5s    → spawn_entity hoàn tất (robot_description + controller_manager)
  t=5s    → collision_warning_node (safety.launch.py)
              Publish /cmd_vel_safe, remap sang
              /mecanum_drive_controller/reference_unstamped
  t=7s    → spawner joint_state_broadcaster (trong gazebo.launch.py)
  t=8.5s  → spawner mecanum_drive_controller (trong gazebo.launch.py)
  t=12s   → slam_toolbox (chờ /scan ổn định)
  t=20s   → Nav2 (chờ map→odom TF từ SLAM)

Cách chạy:
  # BƯỚC 1 — Quét map (Nav2 tắt):
  ros2 launch amr_bringup bringup.launch.py nav:=false
  # Sau đó dùng teleop quét, rồi lưu map:
  # ros2 run nav2_map_server map_saver_cli -f ~/amr_ver5.0/maps/warehouse_v5_map

  # BƯỚC 2 — SLAM + Nav2 (map từ slam_toolbox, không cần file):
  ros2 launch amr_bringup bringup.launch.py

  # BƯỚC 3 — Saved map + Nav2 (dùng map đã lưu):
  ros2 launch amr_bringup bringup.launch.py slam:=false map:=/home/user/amr_ver5.0/maps/warehouse_v5_map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim    = get_package_share_directory('amr_simulation')
    pkg_slam   = get_package_share_directory('amr_slam')
    pkg_nav    = get_package_share_directory('amr_navigation')
    pkg_safety = get_package_share_directory('amr_safety')

    # FIX: tính đường dẫn nav2_params.yaml tại đây để tránh xung đột
    # với argument 'params_file' của slam.launch.py trong cùng launch context
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────
    slam_arg = DeclareLaunchArgument('slam', default_value='true')
    nav_arg  = DeclareLaunchArgument('nav',  default_value='true')
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description="Path to saved map YAML. Bỏ trống (mặc định) = SLAM mode, "
                    "Nav2 dùng map từ slam_toolbox.")
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    yaw_arg    = DeclareLaunchArgument('yaw',    default_value='0.0')

    # ── 1. Simulation — t=0s ───────────────────────────────────────────────
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw':    LaunchConfiguration('yaw'),
        }.items(),
    )

    # ── 2. Safety node — t=5s ─────────────────────────────────────────────
    # QUAN TRỌNG: Safety node phải khởi động trước khi robot cần di chuyển
    # vì mecanum_drive_controller nhận lệnh qua /cmd_vel_safe (remap sang
    # /mecanum_drive_controller/reference_unstamped)
    safety_launch = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[bringup] t=5s: Khởi động collision_warning_node...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_safety, 'launch', 'safety.launch.py')),
            )
        ],
    )

    # ── 3. SLAM — t=12s ───────────────────────────────────────────────────
    slam_launch = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[bringup] t=12s: Khởi động slam_toolbox...'),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_slam, 'launch', 'slam.launch.py')),
                        launch_arguments={'use_sim_time': 'true'}.items(),
                    )
                ],
                condition=IfCondition(LaunchConfiguration('slam')),
            )
        ],
    )

    # ── 4. Nav2 — t=20s ───────────────────────────────────────────────────
    nav_launch = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[bringup] t=20s: Khởi động Nav2...'),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
                        launch_arguments={
                            'use_sim_time': 'true',
                            'map':          LaunchConfiguration('map'),
                            # FIX: pass params_file tường minh để tránh xung đột
                            # với argument cùng tên trong slam.launch.py
                            'params_file':  nav2_params_file,
                        }.items(),
                    )
                ],
                condition=IfCondition(LaunchConfiguration('nav')),
            )
        ],
    )

    return LaunchDescription([
        slam_arg, nav_arg, map_arg,
        x_pose_arg, y_pose_arg, yaw_arg,
        simulation,
        safety_launch,
        slam_launch,
        nav_launch,
    ])
