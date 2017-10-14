import socket
import simplejson as json
import time
import xbox


joy = xbox.Joystick()         #Initialize joystick

TCP_IP = '192.168.1.115'
TCP_PORT = 26656
BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_IP, TCP_PORT))

while True:
    x   = joy.leftX()        #X-axis of the left stick (values -1.0 to 1.0)
    rt  = joy.rightTrigger() #Right trigger position (values 0 to 1.0)
    lt  = joy.leftTrigger()

    obj = {'rt': rt, 'lt': lt, 'x': x}
    MESSAGE = json.dumps(obj, separators=(',', ':'), sort_keys=True)

    print MESSAGE

    sock.send(MESSAGE)
    data = sock.recv(1024)

    print data

joy.close()