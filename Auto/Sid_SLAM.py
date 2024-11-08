import cv2
import numpy as np
import gtsam
from gtsam import Pose3, Rot3, Point3, BetweenFactorPose3, noiseModel
import open3d as o3d  # Import Open3D for point cloud handling

# Define the dimensions of the checkerboard
CHECKERBOARD_SIZE = (9, 6)  # Adjust to your checkerboard size
square_size = 0.025  # Adjust to your square size in meters

# Create a 3D point vector for the checkerboard corners
objp = np.zeros((1, CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3D point in real world space
imgpoints1 = []  # 2D points in image plane for camera 1
imgpoints2 = []  # 2D points in image plane for camera 2

# Capture images from both webcams
cap1 = cv2.VideoCapture(0)  # Left camera
cap2 = cv2.VideoCapture(1)  # Right camera

# Capture images for calibration
num_images_to_capture = 30
frames_captured = 0
while frames_captured < num_images_to_capture:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Failed to capture images.")
        break

    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret1, corners1 = cv2.findChessboardCorners(gray1, CHECKERBOARD_SIZE, None)
    ret2, corners2 = cv2.findChessboardCorners(gray2, CHECKERBOARD_SIZE, None)

    if ret1 and ret2:
        objpoints.append(objp)
        imgpoints1.append(corners1)
        imgpoints2.append(corners2)

        cv2.drawChessboardCorners(frame1, CHECKERBOARD_SIZE, corners1, ret1)
        cv2.drawChessboardCorners(frame2, CHECKERBOARD_SIZE, corners2, ret2)

        frames_captured += 1

    cv2.imshow('Left Camera', frame1)
    cv2.imshow('Right Camera', frame2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()

# Camera calibration and stereo rectification
ret, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints1, frame1.shape[::-1], None, None)
ret, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints, imgpoints2, frame2.shape[::-1], None, None)

_, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
    objpoints, imgpoints1, imgpoints2, mtx1, dist1, mtx2, dist2, frame1.shape[::-1], flags=cv2.CALIB_FIX_INTRINSIC)

# Stereo rectification transformation
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtx1, dist1, mtx2, dist2, frame1.shape[::-1], R, T)

# Initialize ORB detector
orb = cv2.ORB_create()

# Reopen cameras for rectified capture and feature matching
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

# Initialize GTSAM
graph = gtsam.NonlinearFactorGraph()
initial_estimate = gtsam.Values()

# Unique identifier for pose
pose_id = 0

# Lists for storing point cloud data
point_cloud_data = []

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Failed to capture images.")
        break

    # Undistort and rectify images
    map1x, map1y = cv2.initUndistortRectifyMap(mtx1, dist1, R1, P1, frame1.shape[::-1], cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(mtx2, dist2, R2, P2, frame2.shape[::-1], cv2.CV_32FC1)

    rectified1 = cv2.remap(frame1, map1x, map1y, cv2.INTER_LINEAR)
    rectified2 = cv2.remap(frame2, map2x, map2y, cv2.INTER_LINEAR)

    # Convert to grayscale for feature extraction and matching
    gray_left = cv2.cvtColor(rectified1, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(rectified2, cv2.COLOR_BGR2GRAY)

    # Feature detection and matching using ORB
    keypoints1, descriptors1 = orb.detectAndCompute(gray_left, None)
    keypoints2, descriptors2 = orb.detectAndCompute(gray_right, None)

    # Match features using BFMatcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)

    # Draw matches
    matched_image = cv2.drawMatches(rectified1, keypoints1, rectified2, keypoints2, matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Compute the disparity map
    stereo = cv2.StereoBM_create(numDisparities=16 * 5, blockSize=15)
    disparity = stereo.compute(gray_left, gray_right)

    # Normalize disparity for visualization
    disparity_visual = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    disparity_visual = np.uint8(disparity_visual)

    # Set depth range for filtering (adjust these values as needed)
    min_depth = 0.1  # Minimum depth threshold in meters
    max_depth = 2.0  # Maximum depth threshold in meters

    # Reproject points to 3D
    points_3D = cv2.reprojectImageTo3D(disparity, Q)

    # Extract 3D points and corresponding 2D points for PnP
    if len(matches) > 0:
        src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts_3D = points_3D[src_pts[:, 0, 1].astype(int), src_pts[:, 0, 0].astype(int)]

        # Filter points within depth range
        mask_depth = (dst_pts_3D[:, 2] > min_depth) & (dst_pts_3D[:, 2] < max_depth)
        dst_pts_3D_filtered = dst_pts_3D[mask_depth]
        src_pts_filtered = src_pts[mask_depth]

        # Run solvePnP if enough points are available
        if len(src_pts_filtered) >= 4:
            success, rvec, tvec = cv2.solvePnP(dst_pts_3D_filtered, src_pts_filtered, mtx1, dist1)
            if success:
                # Convert rotation vector to rotation matrix
                R_mat, _ = cv2.Rodrigues(rvec)
                
                # Convert to GTSAM Pose3
                pose = Pose3(Rot3(R_mat), Point3(tvec[0], tvec[1], tvec[2]))
                
                # Add prior and odometry to GTSAM
                if pose_id == 0:
                    graph.add(gtsam.PriorFactorPose3(pose_id, pose, gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
                    initial_estimate.insert(pose_id, pose)
                
                # Increment pose ID for next estimation
                pose_id += 1

                # Add loop closure logic (example using nearest neighbor)
                if pose_id > 0:
                    previous_pose = initial_estimate.atPose3(pose_id - 1)
                    odometry = previous_pose.between(pose)
                    graph.add(gtsam.BetweenFactorPose3(pose_id - 1, pose_id, odometry, odometry_noise))

                # Loop closure detection
                 for id in range(pose_id):
                     other_pose = initial_estimate.atPose3(id)
                     translation_diff = np.linalg.norm(np.array([tvec[0], tvec[1]]) - other_pose.translation()[:2])
                     rotation_diff = np.arccos((Rot3(R_mat).matrix()[:2, :2] * other_pose.rotation().matrix()[:2, :2]).trace() / 2)

                    if translation_diff < translation_threshold and rotation_diff < rotation_threshold:
                        loop_closure_odometry = pose.between(other_pose)
                        graph.add(gtsam.BetweenFactorPose3(id, pose_id, loop_closure_odometry, loop_closure_noise))
                
                # Add the pose to the initial estimate
                frame_points_3D = dst_pts_3D_filtered
                initial_estimate.insert(pose_id, pose)
                pose_id += 1

                # Optimize the graph
                optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
                result = optimizer.optimize()
                
               for pose_id in range(len(result)):
    # Retrieve the pose from the optimized result
                   pose = result.atPose3(pose_id)
                   translation = pose.translation()
                   rotation = pose.rotation().matrix()

    # Use the optimized pose to transform the 3D points from each frame
        for point in frame_points_3D[pose_id]:  # frame_points_3D should contain 3D points for each frame
        # Transform the point using the pose's rotation and translation
            transformed_point = rotation @ point + translation

        # Store the transformed point in the point cloud data list
            point_cloud_data.append(transformed_point)
                
                # Display results (optional)
                for i in range(pose_id):
                    print(f"Pose {i}: {result.atPose3(i).translation()}")

    # Prepare point cloud data
    for pt in dst_pts_3D_filtered:
        point_cloud_data.append(pt)

    # Show the matched image and disparity
    cv2.imshow('Matched Features', matched_image)
    cv2.imshow('Disparity', disparity_visual)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()

# Generate and save the PLY file
def save_point_cloud(points, colors, filename):
    points = np.array(points)
    colors = np.array(colors) / 255.0  # Normalize colors to [0, 1] range

    # Create Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)  # Normalize colors

    # Save to .PLY file
    o3d.io.write_point_cloud(filename, pcd)

# Call the save function
save_point_cloud(point_cloud_data, "point_cloud.ply")
