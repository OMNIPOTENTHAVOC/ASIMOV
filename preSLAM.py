import cv2
import numpy as np

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

cam1.release()
cam2.release()
cv2.destroyAllWindows()

# Camera calibration and stereo rectification
ret, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints1, frame1.shape[::-1], None, None)
ret, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints, imgpoints2, frame2.shape[::-1], None, None)

_, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
    objpoints, imgpoints1, imgpoints2, mtx1, dist1, mtx2, dist2, frame1.shape[::-1], flags=cv2.CALIB_FIX_INTRINSIC_K)

# Stereo rectification transformation
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtx1, dist1, mtx2, dist2, frame1.shape[::-1], R, T, flags=cv2.CALIB_RECTIFY_USE_INTRINSIC)

# Capture a pair of rectified images
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

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

    # ... (Rest of your code for disparity map calculation, 3D point cloud generation, etc.)

    cv2.imshow('Rectified Left', rectified1)
    cv2.imshow('Rectified Right', rectified2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
