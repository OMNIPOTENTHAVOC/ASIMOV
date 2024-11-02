import cv2
import glob

left_images = sorted(glob.glob("Middlebury/scene1/img_left/*.png"))
right_images = sorted(glob.glob("Middlebury/scene1/img_right/*.png"))

for left_img_path, right_img_path in zip(left_images, right_images):
    left_img = cv2.imread(left_img_path, cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread(right_img_path, cv2.IMREAD_GRAYSCALE)

    # Resize images (optional, depending on processing power)
    left_img = cv2.resize(left_img, (640, 480))
    right_img = cv2.resize(right_img, (640, 480))

    # Process with your SLAM pipeline
    process_stereo_images(left_img, right_img)
def disparity_to_depth(disparity, focal_length, baseline):
    return (focal_length * baseline) / (disparity + 1e-6)

disparity_map = cv2.imread("Middlebury/scene1/disp_left/frame1.png", cv2.IMREAD_GRAYSCALE)
depth_map = disparity_to_depth(disparity_map, focal_length=1000, baseline=0.1)
