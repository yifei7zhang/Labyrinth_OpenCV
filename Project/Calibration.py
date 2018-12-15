import serial, time, cv2
from collections import deque
import numpy as np
import imutils

########################################### GLOBAL VARIABLES
rangeL = np.array([0,0,0])               # colour range of the marble Low and High
rangeH = np.array([255,255,255])
FileName = None
mode = 0

if mode == 0:
    FileName = 'Red_Marble_Values.txt'
else:
    FileName = 'Wall_Border_Values.txt'

############################################# Image Processing and OpenCV

def setUpValues():
    FileR = open(FileName, "r")
    for i in range(0, 3):
        rangeL[i] = FileR.readline()
        rangeH[i] = FileR.readline()
    cv2.setTrackbarPos('B_LOW' ,'slider', rangeL[0])
    cv2.setTrackbarPos('B_HIGH', 'slider', rangeH[0])
    cv2.setTrackbarPos('G_LOW', 'slider', rangeL[1])
    cv2.setTrackbarPos('G_HIGH', 'slider', rangeH[1])
    cv2.setTrackbarPos('R_LOW', 'slider', rangeL[2])
    cv2.setTrackbarPos('R_HIGH', 'slider', rangeH[2])
    on_trackbar(0)
    FileR.close()

def writeFile():
    fileW = open(FileName,"w")
    fileW.write(str(rangeL[0])+'\n')
    fileW.write(str(rangeH[0])+'\n')
    fileW.write(str(rangeL[1])+'\n')
    fileW.write(str(rangeH[1])+'\n')
    fileW.write(str(rangeL[2])+'\n')
    fileW.write(str(rangeH[2])+'\n')
    fileW.close()


def destructCam(cam):
    cam.release()
    cam.destroyAllWindows()

def processImage(frame):
    if mode != 1:
        frame = cv2.GaussianBlur(frame, (5, 5), cv2.BORDER_DEFAULT)
    filtered = cv2.inRange(frame, rangeL, rangeH)
    if mode != 1:
        filtered = cv2.erode(filtered, None, iterations=2)
        filtered = cv2.dilate(filtered, None, iterations=2)
    return filtered

def calibrate(i):
    lowerR = np.array([0, 0, 0])
    upperR = np.array([255, 255, 255])
    lowerR[i] = rangeL[i]
    upperR[i] = rangeH[i]
    img = cv2.inRange(frame, lowerR, upperR)
    if i == 0:
        cv2.moveWindow("first",0,0)
        cv2.imshow("first", img)
    if i == 1:
        cv2.moveWindow("second",0,400)
        cv2.imshow("second", img)
    if i == 2:
        cv2.moveWindow("third",450,0)
        cv2.imshow("third", img)
    print(i, lowerR[i], upperR[i])

def startCalibration():
    calibrate(0)
    calibrate(1)
    calibrate(2)
    rangeL[0] = cv2.getTrackbarPos('B_LOW','slider')
    rangeH[0] = cv2.getTrackbarPos('B_HIGH','slider')
    rangeL[1] = cv2.getTrackbarPos('G_LOW','slider')
    rangeH[1] = cv2.getTrackbarPos('G_HIGH','slider')
    rangeL[2] = cv2.getTrackbarPos('R_LOW','slider')
    rangeH[2] = cv2.getTrackbarPos('R_HIGH','slider')

####################### CODE STARTS HERE ###################################

def on_trackbar(val):
    cv2.resizeWindow('slider', 500, 0)
    cv2.moveWindow("slider", 450, 400)
    cv2.imshow('slider', val)

cv2.namedWindow('slider')
cv2.resizeWindow('slider',500,500)
cv2.createTrackbar('B_LOW', 'slider', 0, 255, on_trackbar)
cv2.createTrackbar('B_HIGH', 'slider', 0, 255, on_trackbar)
cv2.createTrackbar('G_LOW', 'slider', 0, 255, on_trackbar)
cv2.createTrackbar('G_HIGH', 'slider', 0, 255, on_trackbar)
cv2.createTrackbar('R_LOW', 'slider', 0, 255, on_trackbar)
cv2.createTrackbar('R_HIGH', 'slider', 0, 255, on_trackbar)


setUpValues()
cap = cv2.VideoCapture(0)
(cameraX, cameraY, cameraW, cameraH) = (100, 50, 440, 360)  # dimensions of rectangle

while True:
    ret, frame = cap.read()



    cv2.rectangle(frame,(cameraX,cameraY),(cameraX+cameraW,cameraY+cameraH),(0,255,0),1) #shows the rectangle size of frame; (x.y) is upper left corner, (x+w, y+h) is lower right, brackets after is RGB values), last value is thickness
    frame = frame[cameraY:cameraY+cameraH, cameraX:cameraX+cameraW]   #region of interest
    startCalibration()
    frame = processImage(frame)
    cv2.imshow("orig", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

writeFile()
destructCam(cap)