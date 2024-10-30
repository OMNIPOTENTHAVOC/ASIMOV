import cv2
import numpy as np
import open3d as o3d

# Checkerboard configuration for calibration
CHECKERBOARD_SIZE = (9, 6)  # Adjust to match your checkerboard
square_size = 0.05  # Real-world size of a square on the checkerboard in meters

# Prepare object points for calibration (checkerboard corner points in 3D space)
objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store 3D and 2D points for calibration
objpoints = []
imgpoints1, imgpoints2 = [], []

# Capture images for calibration from two webcams
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    if not ret1 or not ret2:
        break

    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret1, corners1 = cv2.findChessboardCorners(gray1, CHECKERBOARD_SIZE, None)
    ret2, corners2 = cv2.findChessboardCorners(gray2, CHECKERBOARD_SIZE, None)

    if ret1 and ret2:
        objpoints.append(objp)
        imgpoints1.append(corners1)
        imgpoints2.append(corners2)

        cv2.drawChessboardCorners(frame1, CHECKERBOARD_SIZE, corners1, ret1)
        cv2.drawChessboardCorners(frame2, CHECKERBOARD_SIZE, corners2, ret2)

        cv2.imshow("Camera 1", frame1)
        cv2.imshow("Camera 2", frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()

# Camera calibration and stereo rectification
ret, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints1, gray1.shape[::-1], None, None)
ret, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints, imgpoints2, gray2.shape[::-1], None, None)
_, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
    objpoints, imgpoints1, imgpoints2, mtx1, dist1, mtx2, dist2, gray1.shape[::-1], flags=cv2.CALIB_FIX_INTRINSIC)

# Stereo rectification transformation
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtx1, dist1, mtx2, dist2, gray1.shape[::-1], R, T, flags=cv2.CALIB_ZERO_DISPARITY)

# Initialize ORB for feature matching
orb = cv2.ORB_create(nfeatures=3000, scaleFactor=1.2)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Re-open camera streams for feature matching
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    if not ret1 or not ret2:
        break

    # Detect and compute keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(frame1, None)
    kp2, des2 = orb.detectAndCompute(frame2, None)

    # Match descriptors
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)

    # Draw top 20 matches
    img_matches = cv2.drawMatches(frame1, kp1, frame2, kp2, matches[:20], None, flags=2)
    cv2.imshow("Feature Matches", img_matches)

    # Triangulate matched points
    points1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    points2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    points_4d = cv2.triangulatePoints(P1, P2, points1, points2)
    points_4d /= points_4d[3]  # Convert to homogeneous coordinates

    # Convert to 3D points for visualization
    points_3d = points_4d[:3].T  # Extract x, y, z from homogeneous coordinates

    # Create point cloud for visualization
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    o3d.visualization.draw_geometries([pcd])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
