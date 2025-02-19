import matplotlib as plt
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import sys
import os

# print("Python executable:", sys.executable)
# print("OpenCV version:", cv2.__version__)
# print("PYTHONPATH:", os.environ.get('PYTHONPATH'))
# print("PATH:", os.environ.get('PATH'))

capture = cv2.VideoCapture(0)

if not capture.isOpened():
    print("Cannot open camera")
    exit()

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

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (5,5),0)
    cframe = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(cframe, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(cframe, (i[0], i[1]), 2, (0, 0, 255), 3)
            
    cv2.imshow('images/frame.jpg', cframe)
    cv2.imwrite('images/frame.jpg', cframe)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()
