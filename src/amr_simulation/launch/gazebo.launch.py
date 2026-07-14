#!/usr/bin/env python3
"""
gazebo.launch.py — Gazebo Classic + robot spawn (ver6.0, ros2_control thật)
World: warehouse_v5.world (40×30m, multi-room, 8 pillars, 20+ obstacles)

Pipeline cmd_vel:
  teleop/Nav2 → /cmd_vel → collision_warning_node → /cmd_vel_safe
  → (remap) /mecanum_drive_controller/reference_unstamped → mecanum_drive_controller
  → 4x wheel_joint velocity command (gazebo_ros2_control)

Thứ tự khởi động:
  t=5.0s  spawn_entity (robot_description → Gazebo, kèm ros2_control + controller_manager)
  (OnProcessExit spawn_entity)   → spawner joint_state_broadcaster
  (OnProcessExit joint_state_broadcaster) → spawner mecanum_drive_controller
  FIX: 2 spawner trước đây bấm giờ cố định (t=7s/t=8.5s) — trên máy chậm
  chúng có thể chạy trước khi controller_manager sẵn sàng và fail âm thầm,
  khiến mecanum_drive_controller không bao giờ được load (robot đứng yên
  trong Gazebo dù Nav2 vẫn publish cmd_vel bình thường). Giờ dùng
  RegisterEventHandler(OnProcessExit) để đảm bảo thứ tự đúng bất kể tốc độ máy.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('amr_description')
    pkg_sim  = get_package_share_directory('amr_simulation')
    pkg_gaz  = get_package_share_directory('gazebo_ros')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.10')
    yaw_arg    = DeclareLaunchArgument('yaw',    default_value='0.0')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.join(pkg_sim, 'models'), ':',
            '/usr/share/gazebo-11/models',   ':',
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ]
    )
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=['/usr/share/gazebo-11', ':', os.environ.get('GAZEBO_RESOURCE_PATH', '')]
    )

    world_file = os.path.join(pkg_sim, 'worlds', 'warehouse_v5.world')
    urdf_file  = os.path.join(pkg_desc, 'urdf', 'amr_robot.urdf.xacro')

    robot_desc = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gaz, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'pause': 'false', 'verbose': 'false'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    spawn_robot = TimerAction(period=5.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'amr_robot',
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', LaunchConfiguration('z_pose'),
                '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen',
        )
    ])

    # FIX (thay cho TimerAction(period=7.0)/(period=8.5) cũ): trên máy chậm
    # hoặc world nặng vật lý, spawn_entity/controller_manager có thể chưa sẵn
    # sàng đúng lúc 7s/8.5s — khi đó `spawner` gọi service load_controller sẽ
    # fail (exit non-zero) và mecanum_drive_controller/joint_state_broadcaster
    # KHÔNG BAO GIỜ được load, khiến robot đứng yên tuyệt đối trong Gazebo dù
    # Nav2/cmd_vel vẫn chạy bình thường (xem báo cáo debug "AMR ver6.0 -
    # robot không di chuyển trong Gazebo"). Thay bằng OnProcessExit: mỗi
    # spawner chỉ chạy SAU KHI bước trước đó đã thực sự hoàn tất, bất kể máy
    # nhanh/chậm.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen')

    mecanum_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mecanum_drive_controller'], output='screen')

    # spawn_robot (TimerAction period=5.0) bọc 1 Node spawn_entity.py bên
    # trong — lấy đúng Node đó ra để gắn event handler OnProcessExit.
    spawn_entity_node = spawn_robot.actions[0]

    start_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_resource_path,
        x_pose_arg, y_pose_arg, z_pose_arg, yaw_arg,
        gazebo,
        rsp,
        spawn_robot,
        start_joint_state_broadcaster,
        start_mecanum_controller,
    ])
