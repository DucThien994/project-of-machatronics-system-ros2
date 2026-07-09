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
    # khai báo thư viện, gắn đường dẫn 
    pkg_sim    = get_package_share_directory('amr_simulation')
    pkg_slam   = get_package_share_directory('amr_slam')
    pkg_nav    = get_package_share_directory('amr_navigation')
    pkg_safety = get_package_share_directory('amr_safety')

    # FIX: tính đường dẫn nav2_params.yaml tại đây để tránh xung đột
    # với argument 'params_file' của slam.launch.py trong cùng launch context
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # dùng để chạy có nav2 hoặc không
    # thông số sẽ được thay thế vào trong default value 
    slam_arg = DeclareLaunchArgument('slam', default_value='true')
    nav_arg  = DeclareLaunchArgument('nav',  default_value='true')
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description="Path to saved map YAML. Bỏ trống (mặc định) = SLAM mode, "
                    "Nav2 dùng map từ slam_toolbox.")
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    yaw_arg    = DeclareLaunchArgument('yaw',    default_value='0.0')

    # Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw':    LaunchConfiguration('yaw'),
        }.items(),
    )

    # Safety phải hoạt động trước khi robot di chuyển
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

    # SLAM gửi data lên topic /scan
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

    # Nav2
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
