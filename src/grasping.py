from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')

        self.pose_subscription = self.create_subscription(
            msg_type=Pose,
            topic='/pose_topic',
            callback=self.pose_callback,
            qos_profile=10)
        self.pose_subscription  # prevent unused variable warning

    def turn_on_request(self):  # Method for Task 1 to turn on the robot
        while not self.publishers.wait_for_publisher(
                timeout_sec=1.0):  # Loop every 1 second to wait for the server to become available
            self.get_logger().info(f'service not available, waiting...')
    def pose_callback(self, msg):
        self.get_logger().info(
            f"Received pose message - Position: ({msg.position.x}, {msg.position.y}, {msg.position.z}), "
            f"Orientation: ({msg.orientation.w}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z})")
        self.x_pos = msg.position.x
        self.y_pos = msg.position.y
        self.z_pos = msg.position.z
        self.orientation_w = msg.orientation.w
        self.orientation_x = msg.orientation.x
        self.orientation_y = msg.orientation.y
        self.orientation_z = msg.orientation.z
        bot = InterbotixManipulatorXS("px150", "arm", "gripper")
        bot.arm.go_to_home_pose()
        bot.gripper.release()
        bot.arm.set_ee_pose_components(x=self.x_pos, z=self.z_pos,moving_time=5)
        #bot.gripper.grasp()
        #bot.arm.go_to_home_pose(moving_time=4)
   	#bot.arm.set_single_joint_position("waist", np.radians(120), moving_time=5)
        #bot.arm.set_single_joint_position("shoulder", np.radians(-30), moving_time=5)
        #bot.arm.set_single_joint_position("elbow", np.radians(80), moving_time=5)
        #bot.gripper.release()
        #bot.arm.set_single_joint_position("elbow", np.radians(30),moving_time=5)
        #bot.arm.go_to_home_pose(moving_time=4)
        bot.gripper.grasp()
        bot.arm.go_to_sleep_pose()
            
def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin_once(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
