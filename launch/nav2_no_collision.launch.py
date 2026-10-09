"""Minimal Nav2 launch for controlled MPPI diagnosis without collision monitoring.

Execution chain:
  controller / behavior -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel

This intentionally does not start collision_monitor, route_server,
waypoint_follower, or docking_server.  The velocity smoother remains active
and applies the acceleration and deceleration limits in the parameter file.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace_dir = os.path.dirname(os.path.dirname(__file__))
    default_params_file = os.path.join(
        workspace_dir, 'config', 'nav2_params_offline_livox_50hz.yaml'
    )
    default_rviz_config = os.path.join(workspace_dir, 'rviz', 'offline_nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    log_level = LaunchConfiguration('log_level')

    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    command_to_smoother = tf_remappings + [('cmd_vel', 'cmd_vel_nav')]
    smoother_to_base = tf_remappings + [
        ('cmd_vel', 'cmd_vel_nav'),
        ('cmd_vel_smoothed', 'cmd_vel'),
    ]
    common = {
        'output': 'screen',
        'parameters': [params_file, {'use_sim_time': use_sim_time}],
        'arguments': ['--ros-args', '--log-level', log_level],
    }

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', remappings=command_to_smoother, **common
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', remappings=tf_remappings, **common
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', remappings=command_to_smoother, **common
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', remappings=tf_remappings, **common
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', remappings=smoother_to_base, **common
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
