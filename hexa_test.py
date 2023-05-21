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

first_move = True
ports = pypot.dynamixel.get_available_ports()
print(ports)
dxl_io=pypot.dynamixel.DxlIO(ports[0],baudrate=1000000)
robot = SimpleRobot(dxl_io)
params = Parameters(
    freq=50,
    speed=1,
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

# Defining the functions here so it has visibility over the robot variable

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
    keep_going = True
    toto = [0,0]
    while keep_going:

        if args.mode == "arrow":

            x=0
            z=0
            w=100
            h=30
            direction = math.pi / 2

            keys = p.getKeyboardEvents()
            gestion_handle(keys,robot,x,z,h,w,params,direction,args.mode)

        if args.mode == "body":
            x=0
            z=0
            w=100
            h=30
            direction = math.pi / 2
            keys = p.getKeyboardEvents()
            test = gestion_handle(keys,robot,x,z,h,w,params,direction,args.mode,toto)
            setPositionToRobot(test[0],test[1],0,robot,params,0)    

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


