#!/usr/bin/env python
import cv2
import numpy as np
import argparse
import imutils
from collections import deque

# cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False
pts = deque(maxlen=64)
while (True):

    # Take each frame
    _, img = c.read()
    img = cv2.flip(img, 5)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_color = np.array([110, 50, 50], dtype=np.uint8)
    upper_color = np.array([130, 255, 255], dtype=np.uint8)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)

    imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    _, contours, _ = cv2.findContours(imgray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    erode = cv2.erode(res,None,iterations = 3)
    dilate = cv2.dilate(erode,None,iterations = 10)

    if (len(contours) != 0):
        cnt = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(cnt)

        M = cv2.moments(cnt)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if (radius > 15):
           cv2.circle(img, (int(x), int(y)), int(radius), [0, 0, 255], 2)

    cv2.imshow('img', hsv)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()