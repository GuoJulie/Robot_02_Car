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

    # 2. nav to Points
    goal_poses = []

    goal_pose1 = deepcopy(initial_pose)
    goal_pose1.pose.position.x = 2.0
    goal_poses.append(goal_pose1)

    goal_pose2 = deepcopy(goal_pose1)
    goal_pose2.pose.position.y = -1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = deepcopy(goal_pose2)
    goal_pose3.pose.position.x = 1.0
    goal_pose3.pose.position.y = -2.0
    goal_poses.append(goal_pose3)

    nav_start = nav.get_clock().now()
    nav.followWaypoints(goal_poses) # replace   nav.goToPose(goal_pose1)

    # 3. feedback
    i = 0
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        i = i + 1
        if feedback and i % 5 == 0:
            print('Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
                )
            
            #print('Estimated time of arrival: '
            #       + '{0:.0f}'.format(
            #           Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            #       )
            #       + ' seconds.'
            #      )
            
            # check timeout
            now = nav.get_clock().now()
            if now - nav_start > Duration(seconds=600.0):
                nav.cancelTask()
            
            # nav2 request -> change
            #if now - nav_start > Duration(seconds=35.0):
            #    goal_pose4 = deepcopy(goal_pose3)
            #    goal_pose4.pose.position.x = 1.0
            #    goal_pose4.pose.position.y = -1.0
            #    goal_poses.append(goal_pose4)
            #    nav_start = now
            #    nav.followWaypoints(goal_poses)

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
