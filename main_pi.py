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

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(320, 240))

time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    img = frame.array
    img = cv2.flip(img, 0)
    img = cv2.flip(img, 5)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_color = np.array([20, 100, 100], dtype=np.uint8)
    upper_color = np.array([75, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_color, upper_color)
    res = cv2.bitwise_and(img, img, mask=mask)
    imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    
    allContours, _ = cv2.findContours(imgray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if (contours):
        selectedContours = getcontours(contours, 2)
        approxFactor1 = 0.01*cv2.arcLength(selectedContours[0], True)
        approxPoly1 = cv2.approxPolyDP(cnt[0],approxFactor1,True)
        (x1,y1),(w1,h1),_ = cv2.minAreaRect(approxPoly1)
        
        if (len(selectedContours) > 1):
            approxFactor2 = 0.01*cv2.arcLength(selectedContours[1], True)
            approxPoly2 = cv2.approxPolyDP(selectedContours[1],epsilon2,True)
            (x2,y2),(w2,h2),_ = cv2.minAreaRect(approxPoly2)
        
        if (w1 > 5):
            #box1 = cv2.boxPoints(cv2.minAreaRect(approx1))
            #box1 = np.int0(box1)
            
            if(w2 > 5):
            	  #box2 = cv2.boxPoints(cv2.minAreaRect(approx2))
                #box2 = np.int0(box2)
                #cv2.drawContours(img, [box2],0,(255,255,0),2)
                #cv2.drawContours(img,[approx2], 0,(255,0,0), 2)
                
                if ((w1/h1 > 5 or h1/w1 > 5) or len(approxPoly1) < 7):
                    object1 = "goal"
                    object2 = "ball"
                    #print "Goal Height %d" %int(h) + " Width %d" %int(w) + " Ratio %d" %int(w/h) + "Ball Height %d" %int(h) + " Width %d" % int(w) + " Ratio: %d" %int(w/h)
                    
                else:
                    object1 = "ball"
                    object2 = "goal"
                    #print "Goal Height %d" %int(h1) + " Width %d" %int(w1) + " Ratio %d" %int(w1/h1) + "Ball Height %d" %int(h2) + " Width %d" % int(w2) + " Ratio: %d" %int(w2/h2)
                #cv2.putText(img, shape, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                #cv2.putText(img, shape1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
            else:
                if ((w1/h1 > 5 or h1/w1 > 5) or len(approxPoly1) < 7):
                    object1 = "goal"
                    
                else:
                    object1 = "ball"
                #cv2.putText(img, shape, (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                #cv2.drawContours(img,[box1],0,(0,0,255),2)
                #cv2.drawContours(img,[approx1],0,(0,255,0),2)

    #cv2.imshow('hsv', hsv)
    #cv2.imshow('mask', mask)
    #cv2.imshow('res', res)
    #cv2.imshow('imgray', imgray)
    #cv2.imshow('img', img)

    rawCapture.truncate(0)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()