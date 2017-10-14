#!/usr/bin/env python
import urllib2
import time
import cv2
import numpy as np
from collections import deque
import serial
import socket
import simplejson as json

def fetch_img():
    # download the image, convert it to a NumPy array, and then read
    # it into OpenCV format
    fetch_start_time = time.time()
    resp = urllib2.urlopen("http://192.168.1.114/html/cam_pic.php")
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    #print time.time() - fetch_start_time

    return image

def getcontours(allContours, n):
    contours = sorted(allContours, key=cv2.contourArea, reverse=True)
    return contours[:n]

def video(hsv,mask,img):
    #cv2.imshow('hsv', hsv)
    cv2.imshow('mask', mask)
    #cv2.imshow('res', res)
    #cv2.imshow('imgray', imgray)
    cv2.imshow('img', img)
    return

stage = 0
lastDirection = "full right"
h = 18
s = 100
v = 75

TCP_IP = '192.168.1.114'
TCP_PORT = 26656
BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_IP, TCP_PORT))

while True:
    #v = raw_input("input")

    goal_found = False
    ball_found = False
    goalx = -1
    ballx = -1
    ball_width = -1  

    img = fetch_img()
    IMG_HEIGHT, IMG_WIDTH, _ = img.shape
    blurred = cv2.GaussianBlur(img, (11, 11),0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    lower_color = np.array([h, s, v], dtype=np.uint8)
    upper_color = np.array([40, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    #dst = cv2.fastNlMeansDenoising(mask, None, 7, 21, 9)
    #res = cv2.bitwise_and(img, img, mask=mask)
    
    #imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(mask,5)
    #thresh = cv2.adaptiveThreshold(blur,255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2)
    #ret, thresh = cv2.threshold(blur, 50, 255, 0)
    
    _, allContours, _ = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if (allContours):
        #print "here"
        h2 = 1
        w1 = 1
        w2 = 1

        selectedContours = getcontours(allContours, 2)
        approxFactor1 = 0.01*cv2.arcLength(selectedContours[0], True)
        approxPoly1 = cv2.approxPolyDP(selectedContours[0],approxFactor1,True)
        (x1,y1),(w1,h1),_ = cv2.minAreaRect(approxPoly1)
        if (w1 > 25 or h1 > 25):
            #box1 = cv2.boxPoints(cv2.minAreaRect(approxPoly1))
            #box1 = np.int0(box1)

            if (len(selectedContours) > 1):
                approxFactor2 = 0.01*cv2.arcLength(selectedContours[1], True)
                approxPoly2 = cv2.approxPolyDP(selectedContours[1],approxFactor2,True)
                (x2,y2),(w2,h2),_ = cv2.minAreaRect(approxPoly2)

                
                
                if ((w2 > 25 or h2 > 25) and h1 > 0 and w1 > 0):
                    #box2 = np.int0(box2)
                    #cv2.drawContours(img, [box2],0,(255,255,0),2)
                    cv2.drawContours(img,[approxPoly2], 0,(255,0,0), 1)
                    goal_found = True
                    ball_found = True
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
                    if ((h1 != 0 and w1 != 0 and ((w1/h1 > 2 or h1/w1 > 2) or len(approxPoly1) < 7))):
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
                    goal_found = True
                    goalx = x1
                else:
                    object1 = "ball"
                    ball_found = True
                    ballx = x1
                cv2.putText(img, object1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
            #cv2.drawContours(img,[box1],0,(0,0,255),2)
            cv2.drawContours(img,[approxPoly1],0,(0,255,0),1)
    
    center = IMG_WIDTH/2
    tolerance  = 0.1*IMG_WIDTH

    rt = 0
    lt = 0
    x = 0

    if (stage == 0): #Step 1: Turn till the ball is found
        x = -1.0
        rt = 0.5
    if (ball_found and (stage == 0)):
        stage = 1
    if ball_found and (stage == 1): #Step 2: Go towards the ball
        if ((center - tolerance) < ballx and (center+tolerance) > ballx):
            x = (ballx - center) / center
            rt = 0.5
            print "forward"
        elif ((center + tolerance) < ballx):
            x = (ballx - center) / center
            rt = 1
            print "right"
            lastDirection = "full right"
        elif center-tolerance > ballx:
            x = (ballx - center) / center
            rt = 1.0
            print "left"
            lastDirection = "full left"
    elif (stage ==1):
        if lastDirection == "full left":
            x = -1.0
            rt = 0.5
        elif lastDirection == "full right":
            x = 1.0
            rt = 0.5
    if ball_width >= 150 and stage == 1: #Step 3: Stop when close to the ball
        stage = 2
        print "ball stop"
    if stage == 2:
        print "Thing"
        x = -1.0
        rt = 0.0
        lt = 0.5
    if goal_found and stage ==2:
        stage = 3
    if goal_found and stage == 3:
        if ((center-tolerance) < goalx and (center+tolerance) > goalx):
            x = 0
            lt = 0.5
            rt = 0
        elif (center+tolerance) < goalx:
            x = -0.5
            lt = 1
            rt = 0
            lastDirection = "back left"
        elif (center-tolerance) > goalx:
            x = 0.5
            lt = 1
            rt = 0
            lastDirection = "back right"
    elif stage == 3:
        if lastDirection == "back right":
            x = 1
            lt = 0.5
            rt = 0
        elif lastDirection == "back left":
            x = -1
            lt = 0.5
            rt = 0
    if ball_found and goal_found and stage == 3:
        rt = 0
        lt = 0.3
    if ball_found and goal_found and stage == 3 and ball_width <= 60 :
        stage = 4
        rt = 0
        lt = 0
    if ball_found and goal_found and stage == 4:
        stage = 5
        if ballx < goalx: #ball is left of goal
            print "Turn left"
            x = -1
            rt = 0.5
            if ballx > 450:
                stage = 6
                rt = 0
                lt = 0
        if ballx > goalx: #ball is rigt of goal
            print "Turn Right"
            x = 1
            rt = 0.5
            if ballx < 60:
                stage = 6
                rt = 0
                lt = 0
    #print "ball:" + str(ball_found)
    obj = {'rt': rt, 'lt': lt, 'x': x}
    MESSAGE = json.dumps(obj, separators=(',', ':'), sort_keys=True)

    print MESSAGE

    sock.send(MESSAGE)
    data = sock.recv(1024)
    #print data

    video(hsv, mask,img)

    k = cv2.waitKey(3) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
