import cv2

# Written by GPT

def list_cameras():
    index = 0
    arr = []
    while index < 10:  # Check up to 10 devices
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:  # If the camera opens successfully
            arr.append(index)
            cap.release()
        index += 1
    return arr

available_cameras = list_cameras()
print(f"Available cameras: {available_cameras}")