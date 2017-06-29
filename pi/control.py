#!/usr/bin/env python
import numpy as np
import serial
import socket
import simplejson as json
import time

def send_data(control_array, turn_speed_int, global_speed_int):
    if SERIAL_ENABLED:
        a = np.packbits(control_array, axis=0)
        b = np.append(a, [turn_speed_int, global_speed_int])
        data = bytearray(iter(b))

        ser.write(data)

def send_handshake():
    if SERIAL_ENABLED:
        print "Sending handshake"
        start_time = time.time()
        ser.write(bytearray([127]))

        while True:
            if ser.inWaiting() > 0:
                print "Handshake returned"
		break
            elif time.time() - start_time > 1:
                sys.exit("Handshake failed or connection was lost")
    else:
        print "Serial disabled"

def stop_movement():
    if SERIAL_ENABLED:
        control_array = [1, 0, 1, 0, 1]
        send_data(control_array, 0, 0)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
TCP_IP = "192.168.1.114"
TCP_PORT = 26656
s.bind((TCP_IP, TCP_PORT))

SERIAL_ENABLED = 1

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.050, bytesize=8)

send_handshake()

enable = 1

print "Listening on address " + TCP_IP

s.listen(1)

conn, addr = s.accept()
print 'Connection address: ', addr
last_activity = time.time()

while True:
    if SERIAL_ENABLED and ser.inWaiting() and ser.read == 63:
        # reconnect was requested
        send_handshake()

    data = conn.recv(1024)
    if not data and time.time() - last_activity > 1: break
    print "received data: ", data
    last_activity = time.time()
    conn.send(data)
    
    joy = json.loads(data)

    throttle = joy['rt']
    brake = joy['lt']
    x_axis = joy['x']

    global_speed = int(abs(throttle - brake) * 255)
    move = int(global_speed > 20)
    F_B = int(throttle > brake)

    turn_speed = 255 - int(abs(x_axis) * 200)
    L = int(x_axis < 0)
    R = int(x_axis > 0)

    if F_B == 0 and (L == 1 or R == 1):
        L = int(not bool(L))
        R = int(not bool(R))

    send_data([enable, move, F_B, L, R], turn_speed, global_speed)


    if move == 1 and F_B == 1:
        print "Moving forwards"
    elif move == 1 and F_B == 0:
        print "Moving backwards"

stop_movement()

ser.close()
conn.close()
