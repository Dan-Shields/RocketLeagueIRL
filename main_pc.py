#!/usr/bin/env python
import cv2
import numpy as np
from collections import deque
import serial

ser = serial.Serial('COM3', 1200)

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
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    _, contours, _ = cv2.findContours(imgray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



    if (contours):
    #for cnt in contours:
        cnt = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(cnt)

        center = None

        if (radius > 5):
            cv2.circle(img, (int(x), int(y)), int(radius), [0, 0, 255], 2)
            center = (int(x),int(y))


            if (x < 280):
                print "turning left"
                lastDirection = "3"
                ser.write('3')
            elif (x > 360):
                print "turning right"
                lastDirection = "4"
                ser.write('4')
            else:
                print "going straight"
                ser.write('1')
    else:
        print "ball not found, turning " + lastDirection
        ser.write(lastDirection)




        #pts.appendleft(center)

    # for i in xrange(1, len(pts)):
    #     # if either of the tracked points are None, ignore
    #     # them
    #     if pts[i - 1] is None or pts[i] is None:
    #         continue
    #
    #     # otherwise, compute the thickness of the line and
    #     # draw the connecting lines
    #     thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
    #     cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), thickness)


    #cv2.imshow('hsv', hsv)
    #cv2.imshow('mask', mask)
    #cv2.imshow('res', res)
    #cv2.imshow('imgray', imgray)
    cv2.imshow('img', img)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
ser.write('0')
cv2.destroyAllWindows()