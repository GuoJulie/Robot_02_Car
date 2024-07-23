import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. get dir_path
    juliebot_navigation2_dir = get_package_share_directory('juliebot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') # /opt/ros2/humble/share/nav2_bringup
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz') # /opt/ros2/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

    # 2. set Launch config
    # 2.1 define new params
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration('map', default=os.path.join(juliebot_navigation2_dir, 'maps', 'juliebot_map_01.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration('params_file', default=os.path.join(juliebot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # declare new Launch Params
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true.'),
        launch.DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load.'),
        launch.DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load.'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            
            # replace initial params to new params
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_path,
                'params_file': nav2_param_path
            }.items(),
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

    ])
