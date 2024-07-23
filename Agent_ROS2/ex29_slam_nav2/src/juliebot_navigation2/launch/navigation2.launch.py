import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription # import a "launch" file from another pkg
from launch.launch_description_sources import PythonLaunchDescriptionSource # import a "launch" file from another pkg
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1
    # action: locate pkg path
    package_name_1 = 'juliebot_navigation2'
    package_name_2 = 'nav2_bringup'

    # pkg_share = FindPackageShare(package=package_name).find(package_name)
    juliebot_navigation2_dir = get_package_share_directory(package_name_1)
    nav2_bringup_dir = get_package_share_directory(package_name_2)
    
    # 2
    # action: node config -> declare params, get config_file path
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # use gazebo -> sim environment -> get time through topic "/clock", not system time -> use_sim_time=true
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(juliebot_navigation2_dir, 'maps', 'juliebot_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(juliebot_navigation2_dir, 'param', 'juliebot_nav2.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    print(f"rviz config in {rviz_config_dir}")
    
    # 3
    # action: Start nav2_bringup_launch 
    # pkg: navigation2/nav2_bringup
    # arguments: map_yaml_path, nav2_param_path, use_sim_time
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

    # 4
    # action: start Rviz2
    rviz_node = Node(
    	package='rviz2',
    	executable='rviz2',
    	name='rviz2',
    	arguments=['-d', rviz_config_dir],
    	parameters=[{'use_sim_time': use_sim_time}],
    	output='screen'	
    )
    
    ld = LaunchDescription()
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld
