import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.publisher_ = self.create_publisher(
            msg_type=Pose,
            topic='/pose_topic',
            qos_profile=10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def publish_pose(self):
        while self.publisher_.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscribers...')
            rclpy.spin_once(self, timeout_sec = 0.5)
        pose_msg = Pose()
        pose_msg.position = Point(x=0.3, y=0.0, z=0.055)
        pose_msg.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self.publisher_.publish(pose_msg)
        self.get_logger().info('Publishing pose message')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin_once(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
