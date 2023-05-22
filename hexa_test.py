from utils import *
import sys
from signal import signal, SIGINT
import traceback
import pybullet as p
import math
import pypot.dynamixel
import time
import kinematics
from robot_move import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="arrow", help="test")
args = parser.parse_args()

keep_going = True
toto = [0,0]
up_pressed = False 
first_move = True
gear = 1
Z = 0
sens = "inc"
x=0
z=-100
w=100
h=30

ports = pypot.dynamixel.get_available_ports()
print(ports)
dxl_io=pypot.dynamixel.DxlIO(ports[0],baudrate=1000000)
robot = SimpleRobot(dxl_io)
params = Parameters(
    freq=50,
    speed=3,
    z=-110,
    travelDistancePerStep=80,
    lateralDistance=130,
    frontDistance=87,
    frontStart=32,
    method="minJerk",
)
robot.params = params
robot.init()
robot.enable_torque()
for m in robot.motors():
    print(m)

if args.mode == "DILLIDGAF" or args.mode == "test_pygame" or args.mode == "better_arrow":
    pygame.init()
    window = pygame.display.set_mode((800, 600))
    image = pygame.image.load("effortChart.jpg")
    image_rect = image.get_rect()
    image_rect.center = (800 // 2, 600 // 2)
    mouse_down = False
    mouse_pos = [0,0]
    value1 = 0
    value2 = 0
    window.blit(image, image_rect)
    pygame.display.update()

def shutdown(signal_received, frame):
    print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
    robot.disable_torque()
    print("Done ticking. Exiting.")
    sys.exit()

signal(SIGINT, shutdown)
try:
    print("Setting initial position")
    setPositionToRobot(0, 0, 0, robot, params)
    robot.smooth_tick_read_and_write(3, verbose=False)
    print("Init position reached")
    while keep_going:

        if args.mode == "better_arrow":
            x=0
            z=180
            w=80
            h=80
            value1, value2 = DILLIDGAF(x,z,h,w,params,window, image, image_rect,robot, mouse_down, mouse_pos, args.mode, value1, value2)

        if args.mode == "arrow":

            x=0
            z=180
            w=80
            h=80
            direction = math.pi / 2

            keys = p.getKeyboardEvents()
            button_press(keys,robot,x,z,h,w,params,direction,args.mode)  
        
        if args.mode == "avancer":
            x=0
            z=180
            w=80
            h=80
            direction= math.pi / 2
            avancer(x,z,h,w,direction,params, robot)

        if args.mode == "tourner":
            x=0
            z=180
            w=80
            h=80
            direction= math.pi / 2
            tourner(x,z,h,w,direction,params, robot)

        if args.mode == "body":
            if not first_move:
                toto[0], sens = increment_decrement(-50,50,sens, toto[0])
                toto[1], sens = increment_decrement(-50,50,sens, toto[1])
                setPositionToRobot(toto[0],toto[1],-230,robot,params,0)

        if args.mode == "test_pygame":
            direction = math.pi / 2
            x=0
            z=0
            w=100
            h=30
            up_pressed = test_pygame(robot,x,z,h,w,params,direction, up_pressed)

        if args.mode == "DILLIDGAF":
            mouse_down, mouse_pos = DILLIDGAF(x,z,h,w,params,window, image, image_rect,robot, mouse_down, mouse_pos, args.mode, value1, value2)
    
        
        if first_move :
            robot.smooth_tick_read_and_write(3)
            first_move = False
        else :
            robot.tick_read_and_write()
    
except Exception as e:
    track = traceback.format_exc()
    print(track)
finally:
    shutdown(None, None)


