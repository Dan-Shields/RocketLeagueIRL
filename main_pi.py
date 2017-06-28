#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from collections import deque
import serial

def getcontours(contours, n):
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    return contours[:n]

#TODO: change address to a linux specific one
#ser = serial.Serial('/dev/ttyACM0', 9600)

lastDirection = "5"

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(320, 240))

time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # Take each frame
    img = frame.array
    img = cv2.flip(img, 0)
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
        h = 1
        h1 = 1
        cnt = getcontours(contours, 2)
        epsilon1 = 0.01*cv2.arcLength(cnt[0], True)
        approx1 = cv2.approxPolyDP(cnt[0],epsilon1,True)
        (x,y),(w,h),_ = cv2.minAreaRect(approx1)
        w1 = 0
        if (len(cnt) > 1):
            epsilon2 = 0.01*cv2.arcLength(cnt[1], True)
            approx2 = cv2.approxPolyDP(cnt[1],epsilon2,True)
            (x1,y1),(w1,h1),_ = cv2.minAreaRect(approx2)

        center = None
        
        if (w > 5):
            #box1 = cv2.boxPoints(cv2.minAreaRect(approx1))
            #box1 = np.int0(box1)
            if(w1 > 5):
            	#box2 = cv2.boxPoints(cv2.minAreaRect(approx2))
                #box2 = np.int0(box2)
                #cv2.drawContours(img, [box2],0,(255,255,0),2)
                #cv2.drawContours(img,[approx2], 0,(255,0,0), 2)
                if ((w/h > 5 or h/w > 5) or len(approx1) < 7):
                    shape = "goal"
                    shape1 = "ball"
                    print "Goal Height %d" %int(h) + " Width %d" %int(w) + " Ratio %d" %int(w/h) + "Ball Height %d" %int(h) + " Width %d" % int(w) + " Ratio: %d" %int(w/h)
                else:
                    shape = "ball"
                    shape1 = "goal"
                    print "Goal Height %d" %int(h) + " Width %d" %int(w) + " Ratio %d" %int(w/h) + "Ball Height %d" %int(h1) + " Width %d" % int(w1) + " Ratio: %d" %int(w1/h1)
                cv2.putText(img, shape, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(img, shape1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2) 
            else:
                if ((w/h > 5 or h/w > 5) or len(approx1) < 7):
                    shape = "goal"
                else:
                    shape = "ball"
                cv2.putText(img, shape, (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                #cv2.drawContours(img,[box1],0,(0,0,255),2)
                #cv2.drawContours(img,[approx1],0,(0,255,0),2)

    #cv2.imshow('hsv', hsv)
    cv2.imshow('mask', mask)
    #cv2.imshow('res', res)
    #cv2.imshow('imgray', imgray)
    cv2.imshow('img', img)

    rawCapture.truncate(0)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
ser.write('0')
cv2.destroyAllWindows()
