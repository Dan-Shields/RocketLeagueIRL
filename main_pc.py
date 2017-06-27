#!/usr/bin/env python
import cv2
import numpy as np
from collections import deque

def getcontours(contours, n):
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    return contours[:n]
#ser = serial.Serial('COM3', 1200)

lastDirection = "4"

cap = cv2.VideoCapture(0)
pts = deque(maxlen=64)
while True:

    # Take each frame
    _, img = cap.read()
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
    blurred = cv2.GaussianBlur(imgray, (5,5), 0)   
    ret, thresh = cv2.threshold(blurred, 127, 255, 0)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print(contours)


    if (contours):
    #for cnt in contours:#
        #cnt1 = max(contours, key=cv2.contourArea)
        #contours.remove(max(contours, key=cv2.contourArea))
        #cnt2 = max(contours, key=cv2.contourArea)
        cnt = getcontours(contours, 2)
        print(cnt)
        epsilon1 = 0.01*cv2.arcLength(cnt[0], True)
        approx1 = cv2.approxPolyDP(cnt[0],epsilon1,True)
        #((x, y), radius) = cv2.minEnclosingCircle(cnt)
        x,y,w,h = cv2.boundingRect(approx1)
        w1 = 0
        if (len(cnt) > 1):
            epsilon2 = 0.01*cv2.arcLength(cnt[1], True)
            approx2 = cv2.approxPolyDP(cnt[1],epsilon2,True)
            x1,y1,w1,h1 = cv2.boundingRect(approx2)



        center = None

        if (w > 50):
            #cv2.circle(img, (int(x), int(y)), int(radius), [0, 0, 255], 2)
            #center = (int(x),int(y))
            box1 = cv2.boxPoints(cv2.minAreaRect(approx1))
            box1 = np.int0(box1)
            if (w1> 50): 
                box2 = cv2.boxPoints(cv2.minAreaRect(approx2))
                box2 = np.int0(box2)
                cv2.drawContours(img, [box2],0,(255,255,0),2)
                cv2.drawContours(img,[approx2], 0,(255,0,0), 2)
                if (w1/h1 > w/h):
                    shape1 = "goal"
                    shape = "ball"
                else:
                    shape1 = "ball"
                    shape = "goal"
                cv2.putText(img, shape, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(img, shape1, (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) 
            cv2.drawContours(img,[box1],0,(0,0,255),2)
            cv2.drawContours(img,[approx1],0,(0,255,0),2)
            

                

                  



            """if (x < 220):
                print "turning left"
                lastDirection = "6"
                #ser.write('3')
            elif (x > 420):
                print "turning right"
                lastDirection = "5"
                #ser.write('4')
            else:
                print "going straight"
                #ser.write('1')
        else:
            print "ball not found, turning " + lastDirection
            #ser.write(lastDirection)s"""

    #else:
    #    print "ball not found, turning " + lastDirection
    #    #ser.write(lastDirection)



    for i in xrange(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
        cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), thickness)


    #cv2.imshow('hsv', hsv)
    #cv2.imshow('mask', mask)
    #cv2.imshow('res', res)
    #cv2.imshow('imgray', imgray)
    cv2.imshow('img', img)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
#ser.write('0')
cv2.destroyAllWindows()