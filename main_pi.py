#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from collections import deque
import serial

#TODO: change address to a linux specific one
ser = serial.Serial('/dev/ttyACM0', 9600)

lastDirection = "4"

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # Take each frame
    img = frame.array
    img = cv2.flip(img, 5)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of yellow color in HSV
    lower_color = np.array([20, 100, 100], dtype=np.uint8)
    upper_color = np.array([75, 255, 255], dtype=np.uint8)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)

    imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    contours, _ = cv2.findContours(imgray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ball_radius = 0

    if (contours):
    #for cnt in contours:
        cnt = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(cnt)

        center = None
        
        if (radius > 10):
            cv2.circle(img, (int(x), int(y)), int(radius), [0, 0, 255], 2)
            center = (int(x),int(y))

            if (x < 220):
                print "turning left"
                lastDirection = "3"
                ser.write('3')
            elif (x > 420):
                print "turning right"
                lastDirection = "4"
                ser.write('4')
            else:
                print "going straight"
                ser.write('1')
	else:
	    print "ball not found, turning " + lastDirection
	    ser.write(lastDirection)
    else:
       	print "ball not found, turning " + lastDirection
       	ser.write(lastDirection)

    rawCapture.truncate(0)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
ser.write('0')
cv2.destroyAllWindows()