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
def send_data(control_array, turn_speed_int, global_speed_int):
    if SERIAL_ENABLED:
        a = np.packbits(control_array, axis=0)
        b = np.append(a, [turn_speed_int, global_speed_int])
        data = bytearray(iter(b))

        ser.write(data)

def send_handshake():
    if SERIAL_ENABLED:
        print "Sending handshake"
        start_time = time.time()
        ser.write(bytearray([127]))

        while True:
            if ser.inWaiting() > 0:
                print "Handshake returned"
                break
            elif time.time() - start_time > 1:
                sys.exit("Handshake failed or connection was lost")
    else:
        print "Serial disabled"

def stop_movement():
    if SERIAL_ENABLED:
        control_array = [1, 0, 1, 0, 1]
        send_data(control_array, 0, 0)

IMG_HEIGHT = 240
IMG_WIDTH = 320

SERIAL_ENABLED = False

enable = 1

send_handshake()

if SERIAL_ENABLED:
    ser = serial.Serial('/dev/tty/AMC0', 9600, timeout=0.050, bytesize=8)

camera = PiCamera()
camera.resolution = (IMG_WIDTH, IMG_HEIGHT)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(IMG_WIDTH, IMG_HEIGHT))

h = 18
s = 60
v = 150

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #v = raw_input("input")

    if ser.inWaiting() and ser.read == 63:
        # reconnect was requested
        send_handshake()
            
    goal_found = False
    ball_found = False
  
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

            goalx = 0
            ball_width = 0
            ballx = 0

            if (len(selectedContours) > 1):
                approxFactor2 = 0.01*cv2.arcLength(selectedContours[1], True)
                approxPoly2 = cv2.approxPolyDP(selectedContours[1],approxFactor2,True)
                (x2,y2),(w2,h2),_ = cv2.minAreaRect(approxPoly2)

                goal_found = True
                ball_found = True

                if ((w2 > 4 or h2 > 4) and h1 > 0 and w1 > 0):
                    rawCapture.truncate(0)#box2 = cv2.boxPoints(cv2.minAreaRect(approxPoly2))
                    #box2 = np.int0(box2)
                    #cv2.drawContours(img, [box2],0,(255,255,0),2)
                    cv2.drawContours(img,[approxPoly2], 0,(255,0,0), 1)
                    if ((w1/h1 > 2 or h1/w1 > 2) or len(approxPoly1) < 7):
                        object1 = "goal"
                        object2 = "ball"

                        goalx = x1
                        ballx = x2
                        ball_width = w2

                    else:
                        object1 = "ball"
                        object2 = "goal"

                        goalx = x2
                        ballx = x1
                        ball_width = w1
                        
                    #print "Goal Height %d" %int(h1) + " Width %d" %int(w1) + " Ratio %d" %int(w1/h1) + "Ball Height %d" %int(h2) + " Width %d" % int(w2) + " Ratio: %d" %int(w2/h2)
                    cv2.putText(img, object1, (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(img, object2, (int(x2),int(y2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
                else:
                    if ((h1 != 0 and w1 != 0 and ((w1/h1 > 3 or h1/w1 > 3) or len(approxPoly1) < 7))):
                        object1 = "goal"
                        goal_found = True
                        goalx = x1
                    else:
                        object1 = "ball"
                        ball_found = True
                        ballx = x1
                        ball_width = w1

                cv2.putText(img, object1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                #cv2.drawContours(img,[box1],0,(0,0,255),2)
                cv2.drawContours(img,[approxPoly1],0,(0,255,0),1)
        
            else:
                if ((w1/h1 > 2 or h1/w1 > 2) or len(approxPoly1) < 7):
                    object1 = "goal"
                else:
                    object1 = "ball"
                cv2.putText(img, object1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
            #cv2.drawContours(img,[box1],0,(0,0,255),2)
            cv2.drawContours(img,[approxPoly1],0,(0,255,0),1)
            
    if ball_found and goal_found:
        if ballx > goalx and goalx < 50:
            #turn left until > goal is > 50px from the left
            control_array = [enable, 1, 1, 1, 0]
            send_data(control_array, 128, 255)
        elif ballx > goalx and goalx > 50:
            #move forward
            control_array = [enable, 1, 1, 0, 0]
            send_data(control_array, 0, 128)
        else:
            # always send a command to not move to keep-alive connection
            stop_movement()

    else:
        stop_movement()

    video(hsv, mask,img)
    
    rawCapture.truncate(0)

    k = cv2.waitKey(3) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
