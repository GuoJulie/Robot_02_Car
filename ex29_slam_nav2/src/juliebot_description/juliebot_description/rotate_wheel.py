# To do: 
# create a node(roatate_wheel_node) to replace node(joint_state_publisher_gui) to publish Pose_Wheel

import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # topic interface

# node --> replace node(joint_state_publisher_gui)
class RotateWheelNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"node {node_name} init...")

        # create a pub
        self.pub_joint_states_ = self.create_publisher(JointState, "joint_states", 10) # topic: /joint_states
        
        # initialize topic data
        self._init_joint_states()

        # pub topic with a fixed rate --> pub_frequency
        self.pub_rate = self.create_rate(30) # 30Hz
        self.thread_ = threading.Thread(target=self._callback_thread_pub) # create a thread just for pub_work
        self.thread_.start()

    # initialize topic data
    def _init_joint_states(self):
        # init msg type: JointState
        self.joint_states = JointState()
        
        # 1. header
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ''

        # 2. name
        self.joint_states.name = ['left_wheel_joint', 'right_wheel_joint']

        # 3. position
        self.joint_states.position = [0.0, 0.0]

        # 4. velocity
        self.joint_speeds = [0.0, 0.0] # init left/right wheel speed
        self.joint_states.velocity = self.joint_speeds

        # 5. effort
        self.joint_states.effort = []
    
    # external speed interface --> calculate update Pose_Wheel based on the given velocity
    def update_speed(self, speeds):
        self.joint_speeds = speeds

    # thread just for pub_work
    def _callback_thread_pub(self):
        last_update_time = time.time()

        while rclpy.ok():
            # update topic msg data

            # 1. header
            self.joint_states.header.stamp = self.get_clock().now().to_msg()

            # 3. position
            delta_time = time.time() - last_update_time
            last_update_time = time.time()
            self.joint_states.position[0] += delta_time * self.joint_states.velocity[0]
            self.joint_states.position[1] += delta_time * self.joint_states.velocity[1]

            # 4. velocity
            self.joint_states.velocity = self.joint_speeds

            self.pub_joint_states_.publish(self.joint_states)
            self.pub_rate.sleep() # use Rate to keep a cycle_frequency publishing topic



def main(args=None):
    rclpy.init(args=args)
    node = RotateWheelNode("rotate_wheel_node")
    node.update_speed([15.0, -15.0])
    rclpy.spin(node)
    rclpy.shutdown()