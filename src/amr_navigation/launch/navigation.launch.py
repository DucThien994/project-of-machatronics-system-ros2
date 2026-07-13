#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description="Full path to map YAML. Bỏ trống = SLAM mode (dùng slam_toolbox).")
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)

    # ── Conditions 
    use_saved_map = PythonExpression(
        ["'", LaunchConfiguration('map'), "' != ''"])
    use_slam_map = PythonExpression(
        ["'", LaunchConfiguration('map'), "' == ''"])

    # ── Params rewrite 
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': 'true'},
        convert_types=True)

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        condition=IfCondition(use_saved_map),
        parameters=[configured_params,
                    {'yaml_filename': LaunchConfiguration('map')}])

    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        condition=IfCondition(use_saved_map),
        parameters=[configured_params])

    # luon khoi dong nav2
    # Nav2 publish /cmd_vel → collision_warning_node → /cmd_vel_safe → robot
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')])  # Ép xuất ra cmd_vel_nav

    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server',
        name='smoother_server', output='screen',
        parameters=[configured_params])

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[configured_params])

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')])  # Ép xuất ra cmd_vel_nav

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[configured_params])

    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        parameters=[configured_params])

    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),             # Nhận đầu vào từ controller và behavior
            ('cmd_vel_smoothed', 'cmd_vel'),        # Xuất đầu ra chuẩn bị cho collision_warning_node
        ])

    # SLAM mode lifecycle: KHÔNG quản lý map_server/amcl
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        condition=UnlessCondition(use_saved_map),
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }])

    # Saved-map mode lifecycle: quản lý đầy đủ 8 nodes
    lifecycle_manager_saved = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        condition=IfCondition(use_saved_map),
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }])

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        # Saved-map only
        map_server,
        amcl,
        # Always on
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        # Mode-specific lifecycle
        lifecycle_manager_slam,
        lifecycle_manager_saved,
    ])
