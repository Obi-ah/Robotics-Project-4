import cv2
from apriltag import Detector, DetectorOptions
import pyrealsense2 as rs
import numpy as np

# Extract rotation matrix (3x3) and translation vector (3x1)
R = lambda T: T[:3, :3]
t = lambda T: T[:3, 3]

# Convert rotation matrix to Euler angles (roll, pitch, yaw)
def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

options = DetectorOptions()
detector = Detector(options)

while True:
    frames = pipeline.wait_for_frames()

    color_frame = frames.get_color_frame()

    # align depth frames to color frames
    aligned_frames = align.process(frames)    
    depth_frame = aligned_frames.get_depth_frame()

    # Get active profile from the pipeline
    profile = pipeline.get_active_profile()
    rgb_profile = profile.get_stream(rs.stream.color)

    # Extract intrinsics from the stream profile
    rgb_intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()
    # print(f'depth_intr: {rgb_intrinsics}')
    
    # Obtain intrinsic values
    ppx = rgb_intrinsics.ppx
    ppy = rgb_intrinsics.ppy
    fx = rgb_intrinsics.fx
    fy = rgb_intrinsics.fy
    print('depth intrs',rgb_intrinsics)

    # Convert color data to OpenCV format
    color_image = np.asanyarray(color_frame.get_data())
    imggray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    results = detector.detect(imggray)

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
        print(f'Position vector: \n')

        # Calculate world 3D cooordinates
        X = distance * (center[0] - ppx) / fx
        Y = -distance * (center[1] - ppy) / fy
        Z = distance

        X, Y, Z = rs.rs2_deproject_pixel_to_point(rgb_intrinsics, (center), distance)

        t_vec = np.array([X, Y, Z])
        t_vec = t_vec * 100        #change to centimeters
       # Calculate euler angles
        M = detector.detection_pose(tag, (fx,fy,ppx, ppy))[0]  # compare translation vector to prevailing  one
        euler_angles = rotation_matrix_to_euler_angles(R(M))
        euler_angles_deg = np.degrees(euler_angles)
        theta_x, theta_y, theta_z = euler_angles_deg

        print(f'Position vector (cm): \nX: {t_vec[0]}\nY: {t_vec[1]}\nZ: {t_vec[2]}\n\n, d: {distance}')
        print(f'Euler angles (degrees): \ntheta_x: {theta_x}\ntheta_y: {theta_y}\ntheta_z: {theta_z}\n\n\n')

        text_1 = f"Position:  ({t_vec[0]:.2f},  {t_vec[1]:.2f},  {t_vec[2]:.2f})"
        text_2 = f'Orientation:  ({theta_x:.2f},  {theta_y:.2f},  {theta_z:.2f})'



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
    cv2.putText(frame, text_2, position_2, font, scale, color, thickness)

    cv2.imshow("image", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
