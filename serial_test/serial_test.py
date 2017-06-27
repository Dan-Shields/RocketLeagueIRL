#!/usr/bin/env python
import numpy as np
import serial


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.050, bytesize=8)

a = np.packbits([1,0,1,0], axis=0)
b = np.append(a,[255,128])

data = bytearray(b)

ser.write(data)

while True:
    if ser.inWaiting() != 0:
        print ser.readline()
ser.close()
