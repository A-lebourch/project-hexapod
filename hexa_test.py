import utils
from utils import SimpleRobot, Parameters, setPositionToRobot
import sys
from signal import signal, SIGINT
import traceback
import pybullet as p
from onshape_to_robot.simulation import Simulation
import math
import pypot.dynamixel
import time
import kinematics

def main():
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
    
    def tourner(x,z,h,w,direction):
        alphas1 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
        alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 3, params, direction)

        utils.setAnglesToLeg(alphas1, robot.legs[1])
        utils.setAnglesToLeg(alphas2, robot.legs[2])
        utils.setAnglesToLeg(alphas1, robot.legs[3])
        utils.setAnglesToLeg(alphas2, robot.legs[4])
        utils.setAnglesToLeg(alphas1, robot.legs[5])
        utils.setAnglesToLeg(alphas2, robot.legs[6])

    def avancer(x,z,h,w,direction):
        alphas1 = kinematics.triangle(-x,z,h,w,time.time(), 1, params, direction)
        alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 2, params, direction)
        alphas3 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
        alphas4 = kinematics.triangle(x,z,h,w,time.time()+1.5, 4, params, direction)
        alphas5 = kinematics.triangle(-x,z,h,w,time.time(), 5, params, direction)
        alphas6 = kinematics.triangle(-x,z,h,w,time.time()+1.5, 6, params, direction)

        utils.setAnglesToLeg(alphas1, robot.legs[1])
        utils.setAnglesToLeg(alphas2, robot.legs[2])
        utils.setAnglesToLeg(alphas3, robot.legs[3])
        utils.setAnglesToLeg(alphas4, robot.legs[4])
        utils.setAnglesToLeg(alphas5, robot.legs[5])
        utils.setAnglesToLeg(alphas6, robot.legs[6])

    def circle(x, z, r):
        alphas = kinematics.circle(x, z,    r, time.time(), 3)

        utils.setAnglesToLeg(alphas, robot.legs[1])
        utils.setAnglesToLeg(alphas, robot.legs[2])
        utils.setAnglesToLeg(alphas, robot.legs[3])
        utils.setAnglesToLeg(alphas, robot.legs[4])
        utils.setAnglesToLeg(alphas, robot.legs[5])
        utils.setAnglesToLeg(alphas, robot.legs[6])

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
        while keep_going:
            x=0
            z=180
            w=80
            h=80
            direction= math.pi / 2

            avancer(x,z,h,w,direction)            
            # tourner(x,z,h,w,direction)            
            # circle(x,z,20)            

            if first_move :
                robot.smooth_tick_read_and_write(3)
                first_move = False
            else :
                robot.tick_read_and_write()
        return
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


main()