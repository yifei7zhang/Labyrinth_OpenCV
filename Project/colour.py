import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # pink = np.uint8([[[205,23,33]]])
    # hsv_pink = cv2.cvtColor(pink, cv2.COLOR_BGR2HSV)
    # print(hsv_pink)
    # define range of pink color in HSV
    lower_pink = np.array([2,34,153])
    upper_pink = np.array([53,109,255])

    # Threshold the BGR image to get only pink colors
    mask = cv2.inRange(frame, lower_pink, upper_pink)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()