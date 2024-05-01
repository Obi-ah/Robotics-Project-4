from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

##Pose Limits
## X: 0.26-0.46      Try between 0.26-0.36
## Y: -0.20-0.2      Try between -0.1-0.1
## Z: 0.018-0.25     Try between 0.018-0.15


# X: 
# Y:
# Z:


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
       bot.arm.go_to_home_pose(moving_time=7)
       bot.gripper.release()
       bot.arm.set_ee_pose_components(x=self.x_pos,y=self.y_pos, z=self.z_pos,moving_time=7)
       bot.gripper.grasp()
       bot.arm.set_single_joint_position("shoulder", np.radians(-30),moving_time = 5)
       bot.arm.set_single_joint_position("waist", np.radians(148),moving_time = 5)
       bot.arm.set_single_joint_position("elbow", np.radians(80), moving_time = 5)
       bot.arm.set_single_joint_position("wrist_angle", np.radians(0), moving_time = 5)
       #bot.arm.set_ee_pose_components(x=-0.185, y=0.08, z=0.1,moving_time=7)
       bot.gripper.release()
       bot.arm.go_to_home_pose()
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