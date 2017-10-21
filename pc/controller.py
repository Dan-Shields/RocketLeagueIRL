import socket
import simplejson as json
import time
from sdl2 import *


if SDL_Init(SDL_INIT_GAMECONTROLLER) < 0:
    print SDL_GetError()
    raise RuntimeError

joy = None
print SDL_NumJoysticks()
for i in range(SDL_NumJoysticks()):
    if SDL_IsGameController(i):
        joy = SDL_GameControllerOpen(i)
        print SDL_GameControllerMapping(joy)
        if joy:
            break
if not joy:
    raise RuntimeError


TCP_IP = '192.168.1.116'
TCP_PORT = 26656
BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_IP, TCP_PORT))

while True:
    SDL_GameControllerUpdate()
    leftX = SDL_GameControllerGetAxis(joy, SDL_CONTROLLER_AXIS_LEFTX) / 32768.0
    leftT = SDL_GameControllerGetAxis(joy, SDL_CONTROLLER_AXIS_TRIGGERLEFT) / 32768.0
    rightT = SDL_GameControllerGetAxis(joy, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32768.0

    obj = {'rt': rightT, 'lt': leftT, 'x': leftX}
    MESSAGE = json.dumps(obj, separators=(',', ':'), sort_keys=True)

    print MESSAGE

    sock.send(MESSAGE)
    time.sleep(0.02)


SDL_GameControllerClose(joy)
SDL_Quit()
