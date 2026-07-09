import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
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

    # NOTE: ros2_control spawners removed — robot uses libgazebo_ros_planar_move
    # (holonomic planar_move handles cmd_vel_safe → odom directly,
    #  libgazebo_ros_joint_state_publisher handles /joint_states)

    return LaunchDescription([
        gazebo_model_path,
        gazebo_resource_path,
        x_pose_arg, y_pose_arg, z_pose_arg, yaw_arg,
        gazebo,
        rsp,
        spawn_robot,
    ])
