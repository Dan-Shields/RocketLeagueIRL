#!/usr/bin/env python
import socket
import time

import simplejson as json

import RPi.GPIO as gpio


def forwardLeft():
    left.ChangeDutyCycle(turnSpeed * globalSpeed * 100)
    right.ChangeDutyCycle(globalSpeed * 100)

    gpio.output(1, 0)
    gpio.output(2, 1)
    gpio.output(3, 1)
    gpio.output(4, 0)

def forwardRight():
    left.ChangeDutyCycle(globalSpeed * 100)
    right.ChangeDutyCycle(turnSpeed * globalSpeed * 100)
    
    gpio.output(1, 0)
    gpio.output(2, 1)
    gpio.output(3, 1)
    gpio.output(4, 0)

def backwardLeft():
    left.ChangeDutyCycle(globalSpeed * 100)
    right.ChangeDutyCycle(turnSpeed * globalSpeed * 100)

    gpio.output(1, 1)
    gpio.output(2, 0)
    gpio.output(3, 0)
    gpio.output(4, 1)

def backwardRight():
    left.ChangeDutyCycle(turnSpeed * globalSpeed * 100)
    right.ChangeDutyCycle(globalSpeed * 100)

    gpio.output(1, 1)
    gpio.output(2, 0)
    gpio.output(3, 0)
    gpio.output(4, 1)

def forward():
    left.ChangeDutyCycle(globalSpeed * 100)
    right.ChangeDutyCycle(globalSpeed * 100)

    gpio.output(1, 0)
    gpio.output(2, 1)
    gpio.output(3, 1)
    gpio.output(4, 0)

def backward():
    left.ChangeDutyCycle(globalSpeed * 100)
    right.ChangeDutyCycle(globalSpeed * 100)

    gpio.output(1, 1)
    gpio.output(2, 0)
    gpio.output(3, 0)
    gpio.output(4, 1)

def stationary():
    gpio.output(1, 0)
    gpio.output(2, 0)
    gpio.output(3, 0)
    gpio.output(4, 0)


gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

gpio.setup(1, gpio.OUT)
gpio.setup(2, gpio.OUT)
gpio.setup(3, gpio.OUT)
gpio.setup(4, gpio.OUT)

#left
gpio.setup(26, gpio.OUT)
#right
gpio.setup(23, gpio.OUT)

left = gpio.PWM(26, 5000)
left.start(0)
right = gpio.PWM(23, 5000)
right.start(0)



s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
TCP_IP = "192.168.1.114"
TCP_PORT = 26656
s.bind((TCP_IP, TCP_PORT))

print "Listening on address " + TCP_IP

s.listen(1)

conn, addr = s.accept()
print 'Connection address: ', addr
last_activity = time.time()

while True:
    data = conn.recv(1024)
    if not data and time.time() - last_activity > 1: break
    print "received data: ", data
    last_activity = time.time()
    conn.send(data)
    
    joy = json.loads(data)

    throttle = joy['rt']
    brake = joy['lt']
    x_axis = joy['x']

    globalSpeed = int(abs(throttle - brake) * 255)
    move = int(globalSpeed > 20)
    F_B = int(throttle > brake)

    turnSpeed = 255 - int(abs(x_axis) * 200)
    L = int(x_axis < 0)
    R = int(x_axis > 0)

    if F_B == 0 and (L == 1 or R == 1):
        L = int(not bool(L))
        R = int(not bool(R))

    if move == 1 and F_B == 1:
        print "Moving forwards"
    elif move == 1 and F_B == 0:
        print "Moving backwards"

    if move == 1:
        if F_B == 1:
            if L == 1:
                if R == 1:
                    stationary()
                if R == 0:
                    forwardLeft()
            if L == 0:
                if R == 1:
                    forwardRight()
                if R == 0:
                    forward()
        if F_B == 0:
            if L == 1 and R == 1:
                backward()
            elif L == 0:
                if R == 1: 
                    backwardRight()
                if R == 0:
                    backward()
            else:
                backwardLeft()
    else:
        stationary()

stationary()

conn.close()
