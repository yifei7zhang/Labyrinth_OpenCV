import serial, time, cv2
from collections import deque
import enum
from pynput.keyboard import Key, Listener
import numpy as np
import imutils
from serial import Serial

class Settings(enum.Enum):
    KEYBOARD = 1
    BALANCE = 2

########################################### GLOBAL VARIABLES
arduino = None
msg = None          # msg if data is sent to the ArduinoUno
mode = Settings.KEYBOARD
queue_size = 64     # size of queue containing positions of the marble
computer = 1        # 1 is windows port, 2 is dongle port
# 1 is my windows port
# 2 is your dongle port

(cameraX, cameraY, cameraW, cameraH) = (140, 80, 390, 310)  # dimensions of rectangle
circle = None
pos = None  # should store the position of the marble
coordinateMax = np.array([500,500])
marbleCoord = np.array([0,0])

rangeL = np.array([5,21,99])               # colour range of the marble Low and High
rangeH = np.array([67,62,197])
wallL = np.array([0,0,0])
wallH = np.array([255,255,255])

velHighX=0
velHighY=0
xInfo=0
yInfo=0
fractionX=0.7
fractionY=0.5
aborted=False

FileName = 'Red_Marble_Values.txt'
WallFile = 'Wall_Border_Values.txt'

keysPressed = np.zeros(128)                 # boolean array storing which keys are currently being pressed
for i in range(0, 128):
    keysPressed[i] = False

pts = deque(maxlen=queue_size)  #queue containing positions of marble, up to maxsize of queue (in our case 64 points)
velocity = deque(maxlen=queue_size)
accel = deque(maxlen=queue_size)

def getColourRange():
    FileR = open(FileName, "r")
    for i in range(0, 3):
        rangeL[i] = FileR.readline()
        rangeH[i] = FileR.readline()
    FileR.close()
    FileR = open(WallFile, "r")
    for i in range(0, 3):
        wallL[i] = FileR.readline()
        wallH[i] = FileR.readline()
    FileR.close()

########################################### Reading Inputs

def on_press(key):
    try:
        keysPressed[ord(key.char)] = True
    except AttributeError:
        print(".")

def on_release(key):
    keysPressed[ord(key.char)] = False

listener = Listener(on_press=on_press, on_release=on_release)   # Listener for Key inputs
listener.start()

def readKeyInputs():
    s = ""
    if keysPressed[ord('w')]:
        s += 'w'
    if keysPressed[ord('a')]:
        s += 'a'
    if keysPressed[ord('s')]:
        s += 's'
    if keysPressed[ord('d')]:
        s += 'd'
    return s

############################################# Image Processing and OpenCV

def destructCam(cam):
    cam.release()
    cam.destroyAllWindows()

def crop(frame):
    cv2.rectangle(frame,(cameraX,cameraY),(cameraX+cameraW,cameraY+cameraH),(0,255,0),1) #shows the rectangle size of frame; (x.y7uMPing7eh0shaph@s
    # ) is upper left corner, (x+w, y+h) is lower right, brackets after is RGB values), last value is thickness
    frame = frame[cameraY:cameraY+cameraH, cameraX:cameraX+cameraW]   #region of interest
    return frame

def processImage(frame):
    frame = cv2.GaussianBlur(frame, (5, 5), cv2.BORDER_DEFAULT)
    filtered = cv2.inRange(frame, rangeL, rangeH)
    filtered = cv2.erode(filtered, None, iterations=2)
    filtered = cv2.dilate(filtered, None, iterations=2)
    return filtered

def sendMessage():
    byte = 0
    if msg == "w":
        byte |= (0x07)
    elif msg == "s":
        byte |= (0x03)
    if msg == "d":
        byte |= (0x70)
    elif msg == "a":
        byte |= (0x30)
    #print(byte)
    arduino.write(chr(byte).encode("ascii", "ignore"))

####################### CODE STARTS HERE ###################################

def on_trackbar(val):
    cv2.resizeWindow('slider',500,500)
    cv2.imshow('slider', val)

def pixelInRange(arr):
    for i in range(0,3):
        if arr[i]>wallH[i] or arr[i]<wallL[i]:
            return False
    return True

def findMarblePos(frame, filtered):
    global circle,pos,pts
    circle = cv2.findContours(filtered.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circle = circle[0] if imutils.is_cv2() else circle[1]  # version control, deals with opencv2 (Don't look at it)
    if len(circle) > 0:
        c = max(circle, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        pos = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 4:
            cv2.circle(frame, (int(x), int(y)), int(radius), (1, 255, 255), 2)
            cv2.circle(frame, pos, 5, (0, 0, 255), -1)
    if len(pts) >= 1 and pos[0] is not None and pos[1] is not None:
        if abs(pos[0] - pts[0][0]) < 50 and abs(pos[1] - pts[0][1]) < 50:
            pts.appendleft(pos)  # adds the coordinates of the marble into the queue
        """bot=0
        right=0
        left=0
        top=0
        for i in range(0,pos[0]):
            if pixelInRange(frame[pos[1]][pos[0]-i]):
                left = pos[0] - i
                break
        for i in range(0,cameraW-pos[0]-1):
            if pixelInRange(frame[pos[1]][pos[0]+i]):
                right = pos[0] + i
                break
        for i in range(0, pos[1]):
            if pixelInRange(frame[pos[1]-i][pos[0]]):
                top = pos[1] - i
                break
        for i in range(0, cameraH - pos[1]-1):
            if pixelInRange(frame[pos[1]+i][pos[0]]):
                bot = pos[1] + i
                break

        print(str(left)+" "+str(right)+" "+str(top)+" "+str(bot))
        w = float(right-left)
        h = float(bot-top)
        marbleCoord[0]=int(float(pos[0]-left)/w*coordinateMax[0])
        marbleCoord[1]=int(float(pos[1]-top)/h*coordinateMax[1])
        print("Before= "+ str(pos[0])+" "+str(pos[1]))
        print("Loc= " + str(marbleCoord[0])+" "+str(marbleCoord[1]))"""

def fullReset(pos):
    global pts, velocity, accel
    pts.clear()
    if pos is not None:
        pts.appendleft(pos)
    velocity.clear()
    accel.clear()
    print('r')

def kinematics():
    global pts, velocity, accel
    if len(pts) > 1:
        velocity.appendleft([pts[0][0]-pts[1][0],pts[0][1]-pts[1][1]])
    if len(velocity) > 1:
        accel.appendleft([velocity[0][0]-velocity[1][0],velocity[0][1]-velocity[1][1]])

def isMoving():
    global pts
    if len(pts) > 1:        #makes sure at least 2 values are in queue (so compare can work)
        if pts[0] is None or pts[1] is None:    #makes sure we're not comparing when there is no coordinate existing
            return False
        elif (abs(pts[0][0] - pts[1][0]) < 2) and (abs(pts[0][1] - pts[1][1]) < 2): #buffer value of 5 pixels
            return False
        else:
            return True
    return False


def balance():
    global xInfo, yInfo, velHighX, velHighY
    if velocity[0][0]*velHighX<=0 or abs(velHighX)<abs(velocity[0][0]):
        velHighX = velocity[0][0]

    if velocity[0][1]*velHighY<=0 or abs(velHighY)<abs(velocity[0][1]):
        velHighY = velocity[0][1]

    if len(accel) < 1:
        return
    # Horizontal direction
    if velocity[0][0] < 0:
        if abs(float(velHighX*fractionX))>abs(float(velocity[0][0])) and velHighX*velocity[0][0]>0:
            print("t")
            xInfo = -2
        elif accel[0][0] <= 0:
            xInfo = 2
        elif accel[0][0] > 20:
            xInfo = -2
    elif velocity[0][0] > 0:
        if pts[0][0]>320:
            xInfo = -2
        elif abs(float(velHighX*fractionX))>float(abs(velocity[0][0])) and velHighX*velocity[0][0]>0:
            print("t")
            xInfo = 2
        elif accel[0][0] >= 0:
            xInfo = -2
        elif accel[0][0] < -20:
            xInfo = 2

    # Vertical direction
    if velocity[0][1] < 0:
        if abs(float(velHighY*fractionY))>abs(float(velocity[0][1])) and velHighY*velocity[0][1]>0:
            yInfo = -2
        elif accel[0][1] <= 0:
            yInfo = 2
        elif accel[0][1] > 10:
            yInfo = -2
    elif velocity[0][1] > 0:
        if pts[0][0]>275:
            yInfo = -2
        if abs(float(velHighY*fractionY))>abs(float(velocity[0][1])) and velHighY*velocity[0][1]>0:
            yInfo = 2
        elif accel[0][1] >= 0:
            yInfo = -2
        elif accel[0][1] < -20:
            yInfo = 2

def sendDataB():
    byte = 0
    if abs(xInfo) == 2:
        byte |= 0x30
    elif abs(xInfo) == 1:
        byte |= 0x10
    if xInfo > 0:
        byte |= 0x40
    if abs(yInfo) == 2:
        byte |= 0x03
    elif abs(yInfo) == 1:
        byte |= 0x01
    if yInfo < 0:
        byte |= 0x04
    arduino.write(chr(byte).encode("ascii", "ignore"))



############################################## CODE

if computer == 1:
    print("Windows Computer")
    arduino = serial.Serial('COM3', 9600)  # Windows Version
else:
    print("Mac Computer")
    #arduino = serial.Serial('/dev/tty.usbmodem15111', 9600)   '#'#Mac Version
    #arduino = serial.Serial('/dev/tty.usbmodem14111', 9600) # Mac Versiondd
time.sleep(2)

img = np.zeros((300,512,3), np.uint8)
cap = cv2.VideoCapture(1)

getColourRange()

while True:
    ret, frame = cap.read()

    # Fiddling with the Image
    frame = crop(frame)
    filtered = processImage(frame)
    findMarblePos(frame, filtered)
    if keysPressed[ord('1')]:
        mode=Settings.KEYBOARD
        fullReset(pos)
    elif keysPressed[ord('2')]:
        mode=Settings.BALANCE
    if keysPressed[ord('r')]:
        fullReset(pos)
    if keysPressed[ord('x')]:
        aborted = True
    """
    if len(velocity)>0:
        print(velocity[0])
    """
    kinematics()


    xInfo=0
    yInfo=0

    if mode == Settings.BALANCE:
        if isMoving():
            balance()
        if aborted:
            xInfo=0
            yInfo=0
        if len(accel)>0:
            print("X:{} v:{} a:{} vm:{} F:{}".format(pos[1],velocity[0][0],accel[0][0],velHighX, xInfo))
        sendDataB()


    cv2.imshow("orig", frame)

    if mode == Settings.KEYBOARD:
        msg = readKeyInputs()
        sendMessage()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

destructCam(cap)