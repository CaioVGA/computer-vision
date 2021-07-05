# Author Caio Vinicius Gomes Araujo
# Code to transmit the video through the webcam in real time,
# plotting in grayscale and delimiting the edges of an object and figure.

# import rospy
# from cv_bridge import CvBridge
import cv2
import imutils
from imutils.video import VideoStream, FPS
import numpy as np
import argparse
import time

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

# define functions which will be initialized in main
def convert_rgb_to_gray(video_rgb, show):
    gray_image = cv2.cvtColor(video_rgb, cv2.COLOR_BGR2GRAY)
    if show:
        cv2.imshow("Gray Image", gray_image)
    return gray_image

def convert_to_binary(video, adaptative, show):
    if adaptative:
        binary_image = cv2.adaptiveThreshold(video,
                                            255,
                                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY_INV, 5, 2)
    else:
        _,binary_image = cv2.threshold(video, 127, 255, cv2.THRESH_BINARY)
    if show: 
        cv2.imshow("Binary Image", binary_image)
    return binary_image

def getContours(bin_image):
    contours, hierarchy = cv2.findContours(bin_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours, hierarchy

def draw_contours(image, contours, image_name):
    index = -1
    thickness = 2
    color_rgb = (255, 0, 255)
    cv2.drawContours(image, contours, index, color_rgb, thickness)
    cv2.imshow(image_name, image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00']!=0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy

def process_contours(binary, rgb_img, contours):
    black_img = np.zeros(contours[binary.shape[0], binary.shape[1],3], "Contours")
    
    for cont in contours:
        area = cv2.contourArea(cont)
        perimeter = cv2.arcLength(cont, True)
        ((x, y), radius) = cv2.minEnclosingCircle(cont)
        if (area > 10):
            cv2.drawContours(rgb_img, [cont], -1, (150,250,150), 1)
            cv2.drawContours(black_img, [cont], -1, (150,250,150), 1)
            cx, cy = get_contour_center(cont)
            cv2.circle(rgb_img, (cx, cy), (int)(radius), (0,0,255), 1)
            cv2.circle(black_img, (cx, cy), (int)(radius), (0,0,255), 1)
        print("Area: {}, Perimeter: {}".format(area, perimeter))
    print("Countors Count: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours", rgb_img)
    cv2.imshow("Black Image Contours", black_img)

# Implements the functions and call in a main func
def main():
    # ros_pub = rospy.Publisher('/cam_info', Image)
    video = VideoStream(src=0).start()
    time.sleep(2.0)
    fps = FPS().start()
    while True:
        frame = video.read()
        frame = cv2.flip(frame, 1)
        frame = imutils.resize(frame, width=600)  # alocando e definindo os frames em 600 pixels na camera
        frame_gray = convert_rgb_to_gray(frame, True)
        # frame = np.dstack([frame, frame, frame])
        frame_binary = convert_to_binary(frame_gray, True, True)
        frame_binary_basic = convert_to_binary(frame_gray, False, False)
        contornos = getContours(frame_binary)
        # draw_contours(image=frame, contours=contornos, image_name="RGB Image Contours")
        cv2.imshow("Motion Image", frame_binary_basic)
        key = cv2.waitKey(1) & 0xFF
        if cv2.waitKey(1) & key == ord("q"):
            break
        # update the FPS counter
        fps.update()
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    video.release()

if __name__ == '__main__':
    main()