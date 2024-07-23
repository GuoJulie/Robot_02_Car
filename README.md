# Robot_02_Car
## Function: 
- SLAM mapping simulation
- Nav2 point/waypoint navigation simulation
- Automatically identify road conditions and avoid obstacles

## Scenarios:
- Mobility -> Driverless
- AGVs:
  Industrial Logistics - Material Handling,
  Special Environments - Safe Operation,
  Service industry - delivery of food, cleaning, delivery of medicines
  …
  
## Configuration:
### Physical Configuration:
- ESP32 Main Control Board, Radar Adapter Board, Radar, Ultrasonic, Encoders, IMUs, OLEDs, motors and other accessories
### Operating system:
- Ubuntu 22.04 (virtual machine)
### Programming language:
- Python, C++, C
### Frameworks/Platforms/Components:
- Robotics software development framework: ROS2 (+ MicroROS)
- Hardware development framework: Arduino
- Tool visualizer: Rviz2
- Robot Navigation Framework: Nav2
- Code Editor: VsCode
- Internet of Things (IoT) development platform: PlatformIO



# Step:

## [1] 实体机器人：
path_firmware: ~/juliebot/Juliebot_Motion_Control_MicroROS

path_software: ~/juliebot/ex28_juliebot_ws

### 1. ROS2 TF 广播 -> juliebot_bringup
cd /home/ros2/Desktop/juliebot/ex28_juliebot_ws/

// colcon build
source install/setup.bash
ros2 launch juliebot_bringup juliebot_bringup.launch.py 	//坐标变换TF广播

ros2 run rqt_tf_tree rqt_tf_tree 	//坐标变换TF监听
// or 	rqt -> Plugins -> Visualization -> TF Tree

### 2. Agent 连接 -> Connection between ROS2 and MicroROS
agent_wifi: 
	sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6

// rst Juliebot

### 3. 启动MicroROS节点
power on Juliebot

### 4. Juliebot publish Odom -> /topic_odom
ros2 topic echo /topic_odom
ros2 topic hz /topic_odom
ros2 topic bw /topic_odom

### 5. 启动雷达 -> Juliebot laser
cd /home/ros2/Desktop/juliebot/ex28_juliebot_ws/src/ydlidar_ros2/

python3 lidar_wifi.py

// colcon build
source install/setup.bash
ros2 launch ydlidar ydlidar_launch.py

ros2 topic list
ros2 topic echo /scan

rviz2
change: fixedframe = laser_frame
add visualization module by topic /scan
change QOS: 
	LaserScan -> Topic -> Reliability Policy -> Best effort

OR:
	using DOCKER:
	xhost + && sudo docker run  -it --rm  -v /dev:/dev -v /dev/shm:/dev/shm --privileged  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY -p 8889:8888 registry.cn-hangzhou.aliyuncs.com/fishros/fishbot_laser

### 6. 建立地图
<1> using slam_toolbox
ros2 launch slam_toolbox online_async_launch.py

or
<2> using cartegrapher

### 7. 地图可视化 -> Rviz2
打开 RViz，修改 Fixed Frame 为 map，接着通过 Add/By Topic 添加 /map 话题，也可以添加 TF 和 RobotModel 等

### 8. 移动机器人Juliebot -> Twist - keyboard control: (speed_recomended: 0.1m/s)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 topic list
ros2 topic echo /cmd_vel

### 9. 保存地图
ros2 run nav2_map_server map_saver_cli --help
ros2 run nav2_map_server map_saver_cli -t map -f juliebot_map

-> get:
	.
	├── juliebot_map.pgm 地图的数据文件
	└── juliebot_map.yaml 地图的描述文件
	 
	0 directories, 2 files

### 10. Nav2导航
ros2 launch juliebot_navigation2 navigation2.launch.py


## [2] 虚拟机器人： 
path: ~/juliebot/ex29_slam_nav2

### 1. gazebo模拟机器人 （+ rviz实时显示IMU/Odom等监测数据）：
package: juliebot_description
ros2 launch juliebot_description gazebo.launch.py

### 2. slam定位 + cartographer建图 + rviz实时显示IMU/Odom/运动轨迹 等监测数据：
package: juliebot_cartographer
ros2 launch juliebot_cartographer cartographer.launch.py

### 3. 键盘控制Twist 移动 机器人Juliebot 去建图：(speed_recomended: 0.1m/s)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

### 4. nav2 map_server 保存地图：
src下创建map文件夹：mkdir map
ros2 run nav2_map_server map_saver_cli -t map -f juliebot_map

-> get:
	.
	├── juliebot_map.pgm 地图的数据文件
	└── juliebot_map.yaml 地图的描述文件

### 5. nav2 map_server 加载地图 + rivz2中显示：
<1> scr/map/:
	ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=juliebot_map.yaml
	-> 创建：生命周期节点map_server
	-> 生成：/map_server/transition_event (topic)
<2> 生命周期节点map_server 配置：
	ros2 lifecycle set /map_server configure	-> 生成： /map (topic)
<3> rviz2
	add -> by topic -> map
<4> 生命周期节点map_server 激活：
	ros2 lifecycle set /map_server activate		-> rivz中显示juliebot_map
<5> nav2

### 6. nav2 定点/多点 导航：
<1> launch gazebo:
	source install/setup.bash
	ros2 launch juliebot_description gazebo.launch.py
<2> launch nav2:
	source install/setup.bash
	ros2 launch juliebot_navigation2 navigation2.launch.py
