from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from copy import deepcopy
from geometry_msgs.msg import  PoseStamped



def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    # 1. init pose
    # interface: geometry_msgs/msg/PoseStamped --> A Pose with reference coordinate frame and timestamp
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0

    nav.setInitialPose(initial_pose)

    # 2. nav to Point_1
    goal_pose1 = deepcopy(initial_pose)
    goal_pose1.pose.position.x = 1.5
    nav.goToPose(goal_pose1)

    # 3. feedback
    i = 0
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        i = i + 1
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: '
                   + '{0:.0f}'.format(
                       Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                   )
                   + ' seconds.'
                  )
            
            # check timeout
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                nav.cancelTask()
            
            # nav2 request -> change
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose1.pose.position.x = 3.0
            #     nav.goToPose(goal_pose1)
    
    # 4. return result
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    else:
        print('Goal has an invalid return status!')
    
    nav.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()
