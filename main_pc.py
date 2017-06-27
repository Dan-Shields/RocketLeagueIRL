#!/usr/bin/env python
import cv2
import numpy as np
from collections import deque
import serial

def getcontours(contours, n):
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    return contours[:n]

def send_data(control_array, turn_speed_int, global_speed_int):
    a = np.packbits(control_array, axis=0)
    b = np.append(a, [turn_speed_int, global_speed_int])
    data = bytearray(iter(b))

    ser.write(data)

def send_handshake():
    print "Sending handshake"
    start_time = time.time()
    ser.write(127)
    while True:
        if ser.inWaiting() and ser.read() == 127:
            break
        else if time.time() - start_time > 1:
            sys.exit("Handshake failed or connection was lost")

ser = serial.Serial('COM3', 9600, timeout=0.050, bytesize=8)

send_handshake()

enable = 1

cap = cv2.VideoCapture(0)

IMG_WIDTH = 640
IMG_HEIGHT = 480

cap.set(CV_CAP_PROP_FRAME_WIDTH,IMG_WIDTH);
cap.set(CV_CAP_PROP_FRAME_HEIGHT,IMG_HEIGHT);

pts = deque(maxlen=64)
while True:
    if ser.inWaiting():
        if ser.read == 63:
            # reconnect was requested
            send_handshake()

    ball_found = False
    goal_found = False

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

    if (contours):
    #for cnt in contours:#
        #cnt1 = max(contours, key=cv2.contourArea)
        #contours.remove(max(contours, key=cv2.contourArea))
        #cnt2 = max(contours, key=cv2.contourArea)
        h1 = 1
        h = 1
        cnt = getcontours(contours, 2)
        epsilon1 = 0.01*cv2.arcLength(cnt[0], True)
        approx1 = cv2.approxPolyDP(cnt[0],epsilon1,True)
        #((x, y), radius) = cv2.minEnclosingCircle(cnt)
        (x,y),(w,h),_ = cv2.minAreaRect(approx1)
        w1 = 0
        if (len(cnt) > 1):
            epsilon2 = 0.01*cv2.arcLength(cnt[1], True)
            approx2 = cv2.approxPolyDP(cnt[1],epsilon2,True)
            (x1,y1),(w1,h1),_ = cv2.minAreaRect(approx2)

        center = None

        if (w > 20):
            ballx = 0
            ball_width = 0
            goalx = 0

            #cv2.circle(img, (int(x), int(y)), int(radius), [0, 0, 255], 2)
            #center = (int(x),int(y))
            box1 = cv2.boxPoints(cv2.minAreaRect(approx1))
            box1 = np.int0(box1)
            """if (int(h) == 0):
                continue"""
            if (w1> 20): 
                box2 = cv2.boxPoints(cv2.minAreaRect(approx2))
                box2 = np.int0(box2)
                cv2.drawContours(img, [box2],0,(255,255,0),2)
                cv2.drawContours(img,[approx2], 0,(255,0,0), 2)

                ball_found = True
                goal_found = True

                """if (h1 == 0):
                    continue"""
                """if (w1/h1 > w/h):
                    shape1 = "goal"
                    shape = "ball"
                    print "Goal Height %d" %int(h1) + " Width %d" %int(w1) + " Ratio %d" %int(w1/h1) + "Ball Height %d" %int(h) + " Width %d" % int(w) + " Ratio: %d" %int(w/h)        

                else:
                    shape1 = "ball"
                    shape = "goal" """
                if ((w/h > 5 or h/w > 5) or len(approx1) < 7):
                    shape = "goal"
                    shape1 = "ball"

                    ballx = x1
                    ball_width = w1
                    goalx = x

                    print "Goal Height %d" %int(h1) + " Width %d" %int(w1) + " Ratio %d" %int(w1/h1) + "Ball Height %d" %int(h) + " Width %d" % int(w) + " Ratio: %d" %int(w/h)
                else:
                    shape = "ball"
                    shape1 = "goal"

                    ballx = x
                    ball_width = w
                    goalx = x1

                    print "Goal Height %d" %int(h) + " Width %d" %int(w) + " Ratio %d" %int(w/h) + "Ball Height %d" %int(h1) + " Width %d" % int(w1) + " Ratio: %d" %int(w1/h1)

                cv2.putText(img, shape, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(img, shape1, (int(x1),int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2) 
            else:
                if ((w/h > 5 or h/w > 5) or len(approx1) < 7):
                    shape = "goal"
                    goalx = x
                    goal_found = True
                else:
                    shape = "ball"
                    ballx = x
                    ball_width = w
                    ball_found = True
                cv2.putText(img, shape, (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
            cv2.drawContours(img,[box1],0,(0,0,255),2)
            cv2.drawContours(img,[approx1],0,(0,255,0),2)

    if ball_found and goal_found:
        if ballx > goalx and goalx > IMG_WIDTH - 50:
            control_array = [enable, 1, 1, 0, 1]
            send_data(control_array, 128, 255)
        else if ballx > goalx and goalx < IMG_WIDTH - 50
            control_array = [enable, 1, 1, 0, 0]
            send_data(control_array, 0, 128)
    else:
        # always send a command to not move to keep-alive connection
        control_array = [enable, 0, 1, 0, 1]
        send_data(control_array, 0, 0)




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
