#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from collections import deque
import serial

def getcontours(allContours, n):
    contours = sorted(allContours, key=cv2.contourArea, reverse=True)
    return contours[:n]

def video(hsv,mask,img):
    #cv2.imshow('hsv', hsv)
    cv2.imshow('mask', mask)
    #cv2.imshow('res', res)
    #cv2.imshow('imgray', imgray)
    cv2.imshow('img', img)

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(320, 240))
h = 18
s = 60
v = 150

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #v = raw_input("input")
  
    img = frame.array
    img = cv2.flip(img, 0)
    img = cv2.flip(img, 5)
    #blurred = cv2.GaussianBlur(img, (11, 11),0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    

    lower_color = np.array([h, s, v], dtype=np.uint8)
    upper_color = np.array([35, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    #res = cv2.bitwise_and(img, img, mask=mask)
    
    #imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    #blur = cv2.medianBlur(mask,5)
    #thresh = cv2.adaptiveThreshold(blur,255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2)
    #ret, thresh = cv2.threshold(blur, 50, 255, 0)
    
    allContours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if (allContours):
        h2 = 1
        w1 = 1
        w2 = 1
        selectedContours = getcontours(allContours, 2)
        approxFactor1 = 0.01*cv2.arcLength(selectedContours[0], True)
        approxPoly1 = cv2.approxPolyDP(selectedContours[0],approxFactor1,True)
        (x1,y1),(w1,h1),_ = cv2.minAreaRect(approxPoly1)
        if (w1 > 4 or h1 > 4):
            #box1 = cv2.boxPoints(cv2.minAreaRect(approxPoly1))
            #box1 = np.int0(box1)
            if (len(selectedContours) > 1):
                approxFactor2 = 0.01*cv2.arcLength(selectedContours[1], True)
                approxPoly2 = cv2.approxPolyDP(selectedContours[1],approxFactor2,True)
                (x2,y2),(w2,h2),_ = cv2.minAreaRect(approxPoly2)
                if ((w2 > 4 or h2 > 4) and h1 > 0 and w1 > 0):
                    rawCapture.truncate(0)#box2 = cv2.boxPoints(cv2.minAreaRect(approxPoly2))
                    #box2 = np.int0(box2)
                    #cv2.drawContours(img, [box2],0,(255,255,0),2)
                    cv2.drawContours(img,[approxPoly2], 0,(255,0,0), 1)
                    if ((w1/h1 > 2 or h1/w1 > 2) or len(approxPoly1) < 7):
                        object1 = "goal"
                        object2 = "ball" 
                    else:
                        object1 = "ball"
                        object2 = "goal"
                        
                    #print "Goal Height %d" %int(h1) + " Width %d" %int(w1) + " Ratio %d" %int(w1/h1) + "Ball Height %d" %int(h2) + " Width %d" % int(w2) + " Ratio: %d" %int(w2/h2)
                    cv2.putText(img, object1, (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(img, object2, (int(x2),int(y2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
                else:
                    if ((h1 != 0 and w1 != 0 and ((w1/h1 > 3 or h1/w1 > 3) or len(approxPoly1) < 7))):
                        object1 = "goal"
                    else:
                        object1 = "ball"
                cv2.putText(img, object1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                #cv2.drawContours(img,[box1],0,(0,0,255),2)
                cv2.drawContours(img,[approxPoly1],0,(0,255,0),1)
        
            elif (w1 > 4 or h1 > 4):
                if ((w1/h1 > 2 or h1/w1 > 2) or len(approxPoly1) < 7):
                    object1 = "goal"
                else:
                    object1 = "ball"
                cv2.putText(img, object1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
            #cv2.drawContours(img,[box1],0,(0,0,255),2)
            cv2.drawContours(img,[approxPoly1],0,(0,255,0),1)
            
    video(hsv, mask,img)
    
    rawCapture.truncate(0)

    k = cv2.waitKey(3) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
