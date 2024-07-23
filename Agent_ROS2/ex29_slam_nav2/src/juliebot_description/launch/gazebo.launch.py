import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_in_model = 'juliebot'
    package_name = 'juliebot_description'
    urdf_name = "juliebot_gazebo.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'world/juliebot.world')
    
    # 1 Gazebo
    # 1.1 Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path], 
        output='screen')
    
    # 1.2 Launch the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path],
        output='screen'
    )
    
    # 2 Rviz2
    # 2.1 Start Robot State publisher -> Launch the robot in Rviz2
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )
    
    # 2.2 Start Rviz2
    start_rviz_node = Node(
    	package='rviz2',
    	executable='rviz2',
    	name='rviz2',
    	output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_node)
    ld.add_action(robot_state_publisher_node) 
    #ld.add_action(start_rviz_node)

    return ld
