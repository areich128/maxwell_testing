import matplotlib as plt
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import datetime as dt

import sys
import os
import csv
import math

# print("Python executable:", sys.executable)
# print("OpenCV version:", cv2.__version__)
# print("PYTHONPATH:", os.environ.get('PYTHONPATH'))
# print("PATH:", os.environ.get('PATH'))

capture = cv2.VideoCapture(0)

if not capture.isOpened():
    print("Cannot open camera")
    exit()

# FOR RED MARKER
# lower1 = np.array([0, 100, 100])
# upper1 = np.array([10, 255, 255])
# lower2 = np.array([160,100,20])
# upper2 = np.array([179,255,255])

# BLACK
# lower1 = np.array([0, 0, 0])
# upper1 = np.array([180, 255, 40])
# lower2 = np.array([0, 0, 50])
# upper2 = np.array([180, 50, 150])

lower_black = np.array([0, 0, 0])
upper_black = np.array([255, 255, 75])

lower_white = np.array([0, 0, 180])
upper_white = np.array([255, 255, 255])

def detect_circle(frame, lower1, upper1, lower2, upper2):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detect center of black circle
    mask_black = cv2.inRange(frame_hsv, lower1, upper1)
    blurred_black = cv2.GaussianBlur(mask_black, (9, 9), 0)
    circles_black = cv2.HoughCircles(blurred_black, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=5, maxRadius=100)
    if circles_black is not None:
        print("Black circle detected")
        circles_black = np.int32(np.around(circles_black))
        for i in circles_black[0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        coords_black = circles_black[0][0][:2]

    # Detect center of white circle
    mask_white = cv2.inRange(frame_hsv, lower2, upper2)
    blurred_white = cv2.GaussianBlur(mask_white, (9, 9), 0)
    circles_white = cv2.HoughCircles(blurred_white, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=5, maxRadius=100)
    if circles_white is not None:
        print("White circle detected")
        circles_white = np.int32(np.around(circles_white))
        for i in circles_white[0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        coords_white = circles_white[0][0][:2]
        return coords_white
     
    # See how close the two circles are, if they are pretty much on top of each other, return the coordinates 
    # epsilon = 20
    # if (circles_black is not None) and (circles_white is not None):
    #     if (coords_black[0] < coords_white[0]+epsilon and coords_black[0] > coords_white[0]-epsilon) and (coords_black[1] < coords_white[1]+epsilon and coords_black[1] > coords_white[1]-epsilon):
    #         return coords_black
    
    return None

def detect_marker(frame, object):
    template = cv2.imread(object, cv2.IMREAD_GRAYSCALE)
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(template, None)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp2, des2 = orb.detectAndCompute(gray_frame, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    if des2 is not None:
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        if len(matches) > 10:  # Ensure there are enough matches
            # Get matched keypoints
            src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            # Compute homography (perspective transformation)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if M is not None:
                h, w = template.shape
                pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(pts, M)

                # Draw bounding box around detected object
                frame = cv2.polylines(frame, [np.int32(dst)], True, (0, 255, 0), 3)


    return frame

file = open("data/angle.csv", "w")

while True:
    ret, frame = capture.read()
    if not ret:
        exit()

    height, width, _ = frame.shape
    cx = width // 2
    cy = height // 2
    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
    cv2.line(frame, (cx, cy), (width, cy), (0, 255, 0), 2)

    center = detect_circle(frame, lower_black, upper_black, lower_white, upper_white)
    if center is not None:
        cv2.circle(frame, (center[0], center[1]), 2, (0, 0, 255), 3)
        cv2.line(frame, (cx, cy), (center[0], center[1]), (255, 0, 0), 2)
        angle = math.atan2(-(center[1] - cy), center[0] - cx) * 180 / np.pi
        print(f"{-(center[1] - cy)},{center[0] - cx},{angle}")
        current_time = dt.datetime.now()
        file.write(f"{current_time.strftime("%H:%M:%S.%f")},{angle}\n")

    cv2.imshow('result frame', frame)
    cv2.imwrite('images/frame_detect.jpg', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


file.close()
capture.release()
cv2.destroyAllWindows()

