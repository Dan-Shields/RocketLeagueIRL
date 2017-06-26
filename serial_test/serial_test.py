#!/usr/bin/env python
import numpy as np
import serial


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.050)

data = np.array([True, False, True, False, True, False, True, False], dtype=np.bool)
ser.write(data)

while ser.in_waiting:
    print ser.readline()


ser.close()
