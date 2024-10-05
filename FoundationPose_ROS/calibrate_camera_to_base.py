'''18 bottom, 20 top, 17 right, 19 left'''

import numpy as np
import apriltag
import pyrealsense2 as rs
import signal
import cv2
import sys
np.set_printoptions(suppress=True)


def cleanup():
    cam_pose_arr = np.array(cam_pose).reshape(-1, 4, 4)
    # np.save("cam_robot_calibration_data.npy", cam_pose_arr)
    calibrate(cam_pose_arr)
    pipe.stop()


def calibrate(cam_pose_arr):
    calibrated_transform = cam_pose_arr[0]
    np.save("cam_to_robot_base_calibrated_transform.npy", calibrated_transform)
    print("saved calibrated transform as 'cam_to_robot_base_calibrated_transform.npy'")


def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)


def calculate_cube_orientation(detected_tags):
    if len(detected_tags) == 0:
        return None
    
    for detected_tag in detected_tags:
        tag_id = detected_tag.tag_id
        if tag_id in block_tags:
            pos_res = detector.detection_pose(detected_tag, camera_params=(intr.fx, intr.fy, intr.ppx, intr.ppy), tag_size=block_size, z_sign=1)[0]
            tag_orientation = pos_res[0:3, 0:3]
            return tag_orientation @ block_tags[tag_id] #, pos_res[:3, 3]
        

def calculate_cube_center(detected_tags):
    if len(detected_tags) == 0:
        return None

    cube_centers = []

    for detected_tag in detected_tags:
        tag_id = detected_tag.tag_id
        if tag_id not in block_tags:
            continue  # Skip unrecognized tag

        pos_res = detector.detection_pose(detected_tag, camera_params=(intr.fx, intr.fy, intr.ppx, intr.ppy), tag_size=block_size, z_sign=1)[0]
        tag_position = pos_res[:3, 3]

        # Calculate the cube center position based on the tag's position
        cube_center_position = tag_position + pos_res[0:3, 0:3] @ np.array([0, 0, block_size/2])
        cube_centers.append(cube_center_position)

    if not cube_centers:
        return None

    # Compute the average position of the cube center if multiple tags are detected
    cube_center_average = np.mean(cube_centers, axis=0)

    # print(cube_center_average)

    return cube_center_average



signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

pipe = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

options = apriltag.DetectorOptions(families='tag36h11', nthreads=8)
detector = apriltag.Detector(options)

profile = pipe.start(config)

# Get the color stream intrinsics
color_profile = profile.get_stream(rs.stream.color)
intr = color_profile.as_video_stream_profile().get_intrinsics()
K = np.array([[intr.fx, 0, intr.ppx],
              [0, intr.fy, intr.ppy],
              [0, 0, 1]])

block_size = 0.047

# List of valid tag IDs for the block with predefined rotation matrices
block_tags = {
    17: np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]]),                                           ### NEED TO CORRECT THIS
    18: np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]),       ### THIS IS CORRECT
    19: np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),                                           ### NEED TO CORRECT THIS
    20: np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]),     ### THIS IS CORRECT
    21: np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),       ### THIS IS CORRECT
    22: np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])         # np.eye(3)               # np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])    #np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])#####?
}

block_in_base = np.array([[1, 0, 0, 0.116],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0.038],
                          [0, 0, 0, 1]])        ## Based on the block mounting wrt to the base of the robot. Values input from the CAD model of the mount.


cam_pose = []

while True:
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    results = detector.detect(gray_image)

    cube_center = calculate_cube_center(results)
    cube_orientation = calculate_cube_orientation(results)

    tag_poses = {}
    block_in_camera = None

    if cube_center is None or cube_orientation is None:
        print("calibration april tags on robot base not detected")

    if cube_center is not None and cube_orientation is not None:

        print("collecting calibration data. Press 'Stop Camera to Robot Base Calibration' to complete calibration")

        block_in_camera = np.array([[0, 0, 0, cube_center[0]],
                                    [0, 0, 0, cube_center[1]],
                                    [0, 0, 0, cube_center[2]],
                                    [0, 0, 0, 1]])
        
        block_in_camera[0:3, 0:3] = cube_orientation

        # print("Cube center position: ", cube_center)

        camera_in_block = np.linalg.inv(block_in_camera)

        camera_in_base = block_in_base @ camera_in_block

        print(camera_in_base)

        cam_pose.append(camera_in_base)

        # Plot the block orientation
        # Define the axes length for the visualization
        points = np.float32([[0.03, 0.0, 0.0], [0.0, 0.03, 0.0], [0.0, 0.0, 0.03], [0.0, 0.0, 0.0]]).reshape(-1, 3)
        
        # Convert orientation to rotation vector and project 3D points to 2D
        rotV, _ = cv2.Rodrigues(cube_orientation)
        axisPoints, _ = cv2.projectPoints(points, rotV, cube_center, K, (0, 0, 0, 0))
        
        # Draw the axes on the image
        cv2.line(color_image, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[0].ravel().astype(int)), (0, 0, 200), 3)  # Red X-axis
        cv2.line(color_image, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[1].ravel().astype(int)), (0, 200, 0), 3)  # Green Y-axis
        cv2.line(color_image, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[2].ravel().astype(int)), (200, 0, 0), 3)  # Blue Z-axis

    for i in range(len(results)):
        r = results[i]
        pos_res = detector.detection_pose(r, camera_params=(intr.fx, intr.fy, intr.ppx, intr.ppy), tag_size=block_size, z_sign=1)[0]

        # plotting for all tags 

        # square around the tag
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        cv2.line(color_image, ptA, ptB, (0, 255, 0), 1)
        cv2.line(color_image, ptB, ptC, (0, 255, 0), 1)
        cv2.line(color_image, ptC, ptD, (0, 255, 0), 1)
        cv2.line(color_image, ptD, ptA, (0, 255, 0), 1)

        # circle at tag center
        # (cX, cY) = (int(r.center[0]), int(r.center[1]))
        # cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)

        ## tag frame (z-into the tag, x-right, y-down)
        # points = np.float32([[0.03, 0.0, 0.0], [0.0, 0.03, 0.0], [0.0, 0.0, 0.03], [0.0, 0.0, 0.0]]).reshape(-1, 3)
        # rotV, _ = cv2.Rodrigues(pos_res[:3, :3])
        # axisPoints, _ = cv2.projectPoints(points, rotV, pos_res[:3, -1], K, (0, 0, 0, 0))
        # cv2.line(color_image, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[0].ravel().astype(int)), (0, 0, 255), 3)
        # cv2.line(color_image, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[1].ravel().astype(int)), (0, 255, 0), 3)
        # cv2.line(color_image, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[2].ravel().astype(int)), (255, 0, 0), 3)
    
    # if tag_poses:
    #     print("Tag poses: \n", tag_poses)    

    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)

# finally:
#     cleanup()
