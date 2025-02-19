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
    cv2.imshow('images/frame.jpg', frame)
    cv2.imwrite('images/frame.jpg', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()
