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

lower1 = np.array([0, 100, 100])
upper1 = np.array([10, 255, 255])

lower2 = np.array([160,100,20])
upper2 = np.array([179,255,255])

def detect_circle(frame, lower1, upper1, lower2, upper2):
    # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, lower1, upper1) + cv2.inRange(frame_hsv, lower2, upper2)
    blurred = cv2.GaussianBlur(mask, (9, 9), 0)

    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=5, maxRadius=50)
     
    if circles is not None:
        circles = np.int32(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

        return circles[0][0][:2]
    
    return None

file = open("data/angle.csv", "w")

while True:
    ret, frame = capture.read()
    if not ret:
        exit()
    # cv2.imwrite('images/frame.jpg', frame)
    # plt.imshow(frame)
    # plt.show()

    height, width, _ = frame.shape
    cx = width // 2
    cy = height // 2
    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)

    # bwframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # bwframe = cv2.GaussianBlur(frame, (5,5),0)
    # cframe = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0, :]:
    #         cv2.circle(cframe, (i[0], i[1]), i[2], (0, 255, 0), 2)
    #         cv2.circle(cframe, (i[0], i[1]), 2, (0, 0, 255), 3)
    cv2.line(frame, (cx, cy), (width, cy), (0, 255, 0), 2)

    center = detect_circle(frame, lower1, upper1, lower2, upper2)
    if center is not None:
        cv2.circle(frame, (center[0], center[1]), 2, (0, 0, 255), 3)
        cv2.line(frame, (cx, cy), (center[0], center[1]), (255, 0, 0), 2)
        angle = math.atan2(-(center[1] - cy), center[0] - cx) * 180 / np.pi
        print(f"{-(center[1] - cy)},{center[0] - cx},{angle}")
        current_time = dt.datetime.now()
        file.write(f"{current_time.strftime("%H:%M:%S.%f")},{angle}\n")
            
    cv2.imshow('frame', frame)
    cv2.imwrite('images/frame.jpg', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


file.close()
capture.release()
cv2.destroyAllWindows()

time = []
angleseries = []
with open ('data/angle.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        datetime_data = dt.datetime.strptime(row[0], "%H:%M:%S.%f")
        time.append(datetime_data)
        angleseries.append(float(row[1]))

plt.plot(time, angleseries)
plt.xlabel('Time')
plt.ylabel('Angle')
plt.savefig('images/anglevstime.png')
plt.show()

omegaseries = []
omegaseries.append(0)
for i in range(1, len(angleseries)):
    diff = time[i] - time[i-1]
    diff_secs = diff.total_seconds()
    omegaseries.append((angleseries[i] - angleseries[i-1]) / diff_secs)

plt.plot(time, omegaseries)
plt.xlabel('Time')
plt.ylabel('Angular Velocity')
plt.savefig('images/omegavstime.png')
plt.show()

