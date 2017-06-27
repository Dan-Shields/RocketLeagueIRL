#!/usr/bin/env python
import numpy as np
import serial

ser = serial.Serial('COM3', 9600, timeout=0.050, bytesize=8)

a = np.packbits([1, 0, 1, 0], axis=0)
b = np.append(a, [57, 128])

data = bytearray(iter(b))

ser.write(data)

while True:
    if ser.inWaiting() == 3:
        ser.write(data)
        print ser.read()
        print ser.read()
        print ser.read()

ser.close()
