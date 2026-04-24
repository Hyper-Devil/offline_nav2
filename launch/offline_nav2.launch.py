import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ros2 launch offline_nav2 offline_nav2.launch.py params_file:=/home/ugv/colcon_ws/src/offline_nav2/config/nav2_params_offline_local_only.yaml
def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    workspace_dir = os.path.dirname(os.path.dirname(__file__))
    default_params_file = os.path.join(workspace_dir, 'config', 'nav2_params_offline_local_only.yaml')
    default_rviz_config = os.path.join(workspace_dir, 'rviz', 'offline_nav2.rviz')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='False',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Whether to launch nav2 SLAM. Keep False for offline playback if SLAM is external.'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Map yaml file path. Not used when use_localization is False.'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use /clock from ros bag playback'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Path to the ROS 2 parameters file for Nav2 nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the Nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False', description='Whether to respawn if a node crashes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to launch RViz'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config_file', default_value=default_rviz_config,
        description='Full path to RViz config file'
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'use_localization': 'False',
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)

    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
