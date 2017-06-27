#!/usr/bin/env python
import numpy as np
import serial
import xbox
joy = xbox.Joystick()

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.050, bytesize=8)



a = np.packbits([0, 1, 1, 1, 1, 1, 1, 1], axis=0)
b = np.append(a, [255, 255])

data = bytearray(iter(b))
ser.write(data)
print "Sending handshake"

enable = 1

while not joy.Back():
    throttle = joy.rightTrigger()
    brake = joy.leftTrigger()
    x_axis = joy.leftX()

    global_speed = int(abs(throttle - brake) * 255)
    move = int(global_speed > 20)
    F_B = int(throttle > brake)

    turn_speed = 255 - int(abs(x_axis) * 255)
    L = int(x_axis < 0)
    R = int(x_axis > 0)

    if not joy.B():
        global_speed = int(global_speed / 2)

    a = np.packbits([enable, move, F_B, L, R], axis=0)
    b = np.append(a, [turn_speed, global_speed])

    data = bytearray(iter(b))

    if move == 1 and F_B == 1:
        print "Moving forwards"
    elif move == 1 and F_B == 0:
        print "Moving backwards"

    #write data if previous data was received
    if ser.inWaiting() > 0:
        print "Wrote data"
        ser.write(data)
        ser.read()

a = np.packbits([0], axis=0)
b = np.append(a, [0, 0])

data = bytearray(iter(b))

ser.write(data)

ser.close()
joy.close()
