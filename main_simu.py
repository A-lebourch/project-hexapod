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
parser.add_argument("--mode", "-m", type=str, default="arrow", help="test")
args = parser.parse_args()

first_move = True
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
#? sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)

sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

robot = SimpleRobotSimulation(sim)
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
robot.centerCamera = True
robot.init()
robot.enable_torque()
itr  = 1

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

        robot_pose = (
            sim.getRobotPose()
        )  
        yaw = robot_pose[1][2]
        sim.lookAt(robot_pose[0])
        
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

