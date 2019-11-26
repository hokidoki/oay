from picamera.array import PiRGBArray
from picamera import PiCamera
from collections import deque
import time
import cv2
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
AIN1=24
BIN1=21
AIN2=19
BIN2=23

sig=deque([1,0,0,0])
dir=1
GPIO.setup(AIN1,GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(AIN2,GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(BIN1,GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(BIN2,GPIO.OUT, initial=GPIO.LOW)

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.4, minNeighbors=4, minSize=(100,100),
                                     flags=cv2.CASCADE_SCALE_IMAGE)

    if len(rects) == 0:
        return []

    rects[:,2:] += rects[:,:2]
    return rects

def rotate():
    for i in range(3):
        GPIO.output(AIN1, sig[0])
        GPIO.output(BIN1, sig[1])
        GPIO.output(AIN2, sig[2])
        GPIO.output(BIN2, sig[3])
        time.sleep(0.02)
        sig.rotate(-1)

def rerotate():
    for i in range(3):
        GPIO.output(AIN1, sig[0])
        GPIO.output(BIN1, sig[1])
        GPIO.output(AIN2, sig[2])
        GPIO.output(BIN2, sig[3])
        time.sleep(0.02)
        sig.rotate(1)
    

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1,y1), (x2, y2), color, 2)
        xstep = int(((x1+x2)/2))
        ystep = int(((y1+y2)/2))
        if (xstep >= 370 or xstep <= 350):
            if (xstep < 360):
                rotate()
            else:
                rerotate()
            print(xstep)
        #print(ystep)
        #if (ystep >= 250 or ystep <= 230):
        #    if (ystep < 240):
        #        rotate()
        #    else:
        #        rerotate()

camera = PiCamera()
camera.resolution = (720, 480)
camera.framerate = 40
rawCapture = PiRGBArray(camera, size=(720, 480))
cascade = cv2.CascadeClassifier("/home/pi/Downloads/opencv-master/data/haarcascades/haarcascade_frontalface_alt2.xml")

time.sleep(0.05)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    rects = detect(gray, cascade)
    vis = img.copy()
    draw_rects(vis, rects, (0, 255, 0))

    cv2.imshow("Frame", vis)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)

    if key == ord("q"):
        break

GPIO.cleanup()
