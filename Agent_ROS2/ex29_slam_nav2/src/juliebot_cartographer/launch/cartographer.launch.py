import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_in_model = 'juliebot'
    package_name = 'juliebot_cartographer'
    
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # node config
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # use gazebo -> use_sim_time=true
    resolution = LaunchConfiguration('resolution', default='0.05') # map resolution
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0') # map publish period
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(pkg_share, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='juliebot_2d.lua')
    rviz_config_dir = os.path.join(pkg_share, 'config') + "/cartographer.rviz"
    print(f"rviz config in {rviz_config_dir}")
    
    # 1
    # get topic: /scan (node: laserscan, provided by Gazebo)
    # get topic: /odom (node: diff_drive, provided by Gazebo)
    # arguments: config -> 'juliebot_2d.lua'
    # send topic: /submap_list
    cartographer_node = Node(
    	package='cartographer_ros',
    	executable='cartographer_node',
    	name='cartographer_node',
    	output='screen',
    	parameters=[{'use_sim_time': use_sim_time}],
    	arguments=['-configuration_directory', configuration_directory,
    		'-configuration_basename', configuration_basename]
    )
    
    # 2
    # get topic: /submap_list
    # arguments: resolution, publish_period_sec
    # action: merge submap -> map -> publish map
    cartographer_occupancy_grid_node = Node(
    	package='cartographer_ros',
    	executable='cartographer_occupancy_grid_node',
    	name='cartographer_occupancy_grid_node',
    	output='screen',
    	parameters=[{'use_sim_time': use_sim_time}],
    	arguments=['-resolution', resolution,
    		'-publish_period_sec', publish_period_sec]
    )
    
    # 3
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
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
