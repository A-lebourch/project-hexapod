import sys
from utils import *
from signal import signal, SIGINT
import traceback
import pybullet as p
from onshape_to_robot.simulation import Simulation
import math
from robot_move import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="arrow")
args = parser.parse_args()

robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
#? sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
robot = SimpleRobotSimulation(sim)
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
robot.centerCamera = True
robot.init()
robot.enable_torque()

up_pressed = False 
keep_going = True
toto = [0,0,0]
gear = 1
Z = 0
sens = "inc"
x=0
z=0
w=100
h=30
first_move = True

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

        robot_pose = (
            sim.getRobotPose()
        )  
        yaw = robot_pose[1][2]
        sim.lookAt(robot_pose[0])
        
        if args.mode == "arrow":
            direction = math.pi / 2
            keys = p.getKeyboardEvents()
            x,z,h,w,gear=speed(keys,gear)
            button_press(keys,robot,x,z,h,w,params,direction,args.mode)
        
        if args.mode == "better_arrow":
            x=0
            z=0
            w=80
            h=80
            value1, value2 = DILLIDGAF(x,z,h,w,params,window, image, image_rect,robot, mouse_down, mouse_pos, args.mode, value1, value2)


        if args.mode == "DILLIDGAF":
            mouse_down, mouse_pos = DILLIDGAF(x,z,h,w,params,window, image, image_rect,robot, mouse_down, mouse_pos, args.mode, value1, value2)

        if args.mode == "test_pygame":
            direction = math.pi / 2
            x=0
            z=0
            w=100
            h=30
            up_pressed = test_pygame(robot,x,z,h,w,params,direction, up_pressed)
        
        if args.mode == "leg":

            x=200
            y=0
            z, sens = increment_decrement(-100, 50, sens, z)
            setPositionToLeg(200,y,z,robot.legs[1])

        if args.mode == "body":
            x=0
            z=0
            w=100
            h=30
            direction = math.pi / 2
            keys = p.getKeyboardEvents()
            test = button_press(keys,robot,x,z,h,w,params,direction,args.mode,toto)
            setPositionToRobot(test[0],test[1],test[2],robot,params,0)

        if args.mode == "go_to":
            pass

        #? sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))

        if first_move :
            robot.smooth_tick_read_and_write(3)
            first_move = False
        else :
            robot.tick_read_and_write()
        sim.tick()
    
except Exception as e:
    track = traceback.format_exc()
    print(track)
finally:
    shutdown(None, None)

