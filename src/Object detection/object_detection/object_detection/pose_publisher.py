import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
# from detection_main import t_vec
import cv2
from apriltag import Detector, DetectorOptions
import pyrealsense2 as rs
import numpy as np


# Extract rotation matrix (3x3) and translation vector (3x1)
R = lambda T: T[:3, :3]
t = lambda T: T[:3, 3]

# Convert rotation matrix to Quaternion
import numpy as np

def rotation_matrix_to_quaternion(R):

    # Ensure the input matrix is a numpy array
    R = np.array(R)

    # Extract the elements of the rotation matrix
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]

    # Calculate the quaternion components
    qw = np.sqrt(1 + r11 + r22 + r33) / 2
    qx = (r32 - r23) / (4 * qw)
    qy = (r13 - r31) / (4 * qw)
    qz = (r21 - r12) / (4 * qw)

    return np.array([qw, qx, qy, qz])


class PosePublisher(Node):
   def __init__(self):
      super().__init__('pose_publisher')

      self.publisher_ = self.create_publisher(
         msg_type=Pose,
         topic='/pose_topic',
         qos_profile=10)
      timer_period = 0.03  # seconds
      self.timer = self.create_timer(timer_period, self.publish_pose)

      # Configure depth and color streams
      self.pipeline = rs.pipeline()
      self.config = rs.config()
      self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
      self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

      # Start streaming
      self.pipeline.start(self.config)

      align_to = rs.stream.color
      self.align = rs.align(align_to)

      options = DetectorOptions()
      self.detector = Detector(options)

   def publish_pose(self):
      # while self.publisher_.get_subscription_count() == 0:
      #    self.get_logger().info('Waiting for subscribers...')
      #    rclpy.spin_once(self, timeout_sec = 0.5)


      frames = self.pipeline.wait_for_frames()

      aligned_frames = self.align.process(frames)
      depth_frame = aligned_frames.get_depth_frame()
      color_frame = aligned_frames.get_color_frame()

      # Get active profile from the pipeline
      profile = self.pipeline.get_active_profile()
      depth_profile = profile.get_stream(rs.stream.depth)

      # Extract intrinsics from the stream profile
      depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
      
      # Obtain intrinsic values
      ppx = depth_intrinsics.ppx
      ppy = depth_intrinsics.ppy
      fx = depth_intrinsics.fx
      fy = depth_intrinsics.fy
      # print('depth intrs',depth_intrinsics)

      # Convert color data to OpenCV format
      color_image = np.asanyarray(color_frame.get_data())
      imggray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

      results = self.detector.detect(imggray)

      text_1 = ''
      text_2 = ''
      # Mark boundaries and center
      if results:
         tag = results[0]
         ptA, ptB, ptC, ptD  = tag.corners
         
         ptA = (int(ptA[0]), int(ptA[1]))
         ptB = (int(ptB[0]), int(ptB[1]))
         ptC = (int(ptC[0]), int(ptC[1]))
         ptD = (int(ptD[0]), int(ptD[1]))

         cv2.line(color_image, ptA, ptB, (0, 255, 0), 4)
         cv2.line(color_image, ptB, ptC, (0, 255, 0), 4)
         cv2.line(color_image, ptC, ptD, (0, 255, 0), 4)
         cv2.line(color_image, ptD, ptA, (0, 255, 0), 4)

         center = (int(tag.center[0]), int(tag.center[1]))
         cv2.circle(color_image, center, 5, (0, 0, 255), -1)

         # Get distance to center
         distance = depth_frame.get_distance(center[0], center[1])  # may be center[1], center[0]. Test.
         # print(f'Position vector: \n')

         # Calculate world 3D cooordinates
         X = distance * (center[0] - ppx) / fx
         Y = -distance * (center[1] - ppy) / fy
         Z = distance

         # X2, Y2, Z = rs.rs2_deproject_pixel_to_point(depth_intrinsics, (center), distance)

         t_vec = np.array([X, Y, Z])
         t_vec = t_vec * 100        #change to centimeters

         # Calculate euler angles
         M = self.detector.detection_pose(tag, (fx,fy,ppx, ppy))[0]  # compare translation vector to prevailing  one
         # euler_angles = rotation_matrix_to_euler_angles(R(M))
         # euler_angles_deg = np.degrees(euler_angles)
         # theta_x, theta_y, theta_z = euler_angles_deg
         q = rotation_matrix_to_quaternion(R(M))
         print(q)

         pose_msg = Pose()
         pose_msg.position = Point(x=t_vec[0], y=t_vec[1], z=t_vec[2])
         pose_msg.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
         self.publisher_.publish(pose_msg)
         self.get_logger().info('Publishing pose message')
         # print(f'Position vector (cm): \nX: {t_vec[0]}\nY: {t_vec[1]}\nZ: {t_vec[2]}\n\n, d: {distance}')
         # print(f'Euler angles (degrees): \ntheta_x: {theta_x}\ntheta_y: {theta_y}\ntheta_z: {theta_z}\n\n\n')

         text_1 = f"Position:  ({t_vec[0]:.2f},  {t_vec[1]:.2f},  {t_vec[2]:.2f})"
         # text_2 = f'Orientation:  ({theta_x:.2f},  {theta_y:.2f},  {theta_z:.2f})'



      cv2.namedWindow('image', cv2.WINDOW_FREERATIO)
      cv2.resizeWindow('image', 1200, 950)
      cv2.moveWindow("image", 700, 100)

      frame = cv2.flip(color_image, cv2.ROTATE_180)

      position_1 = (50, 50)  # (x, y) coordinates
      position_2 = (50, 70)  # (x, y) coordinates
      font = cv2.FONT_HERSHEY_SIMPLEX
      scale = 0.5
      color = (255, 102, 255)  # BGR color format
      thickness = 1   

      # Add text to the frame
      cv2.putText(frame, text_1, position_1, font, scale, color, thickness)
      # cv2.putText(frame, text_2, position_2, font, scale, color, thickness)

      cv2.imshow("image", frame)

      if cv2.waitKey(1) & 0xFF == ord('q'):
         raise KeyboardInterrupt
       

def main(args=None):
   rclpy.init(args=args)
   pose_publisher = PosePublisher()
   rclpy.spin(pose_publisher)
   pose_publisher.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()