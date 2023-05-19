import utils
from utils import SimpleRobotSimulation, Parameters, setPositionToRobot
import sys
from signal import signal, SIGINT
import traceback
import pybullet as p
from onshape_to_robot.simulation import Simulation
import math
import time
import kinematics
from scipy.spatial.transform import Rotation


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat

def main():
    first_move = True
    robotPath = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    
    # Try to play with the friction parameters of the simulator
    # sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
    
    # Setting the robot's initial position
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
    for m in robot.motors():
        print(m)
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:
        print("Setting initial position")
        
        setPositionToRobot(0, 0, 0, robot, params)
        robot.smooth_tick_read_and_write(3, verbose=False)
        print("Init position reached")
        # You can create a graphical user interface if you want
        # ui = display.RobotUI(robot, params)
        keep_going = True
        while keep_going:
            # keep_going = ui.tick()

            # Getting the robot position and asking the camera to follow
            robot_pose = (
                sim.getRobotPose()
            )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
            yaw = robot_pose[1][2]
            sim.lookAt(robot_pose[0])
            
            x=0
            z=0
            w=100
            h=100
            direction= math.pi / 2

            alphas1 = kinematics.triangle(x,z,h,w,time.time(), 1, params, direction)
            alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 2, params, direction)
            alphas3 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
            alphas4 = kinematics.triangle(x,z,h,w,time.time()+1.5, 4, params, direction)
            alphas5 = kinematics.triangle(x,z,h,w,time.time(), 5, params, direction)
            alphas6 = kinematics.triangle(x,z,h,w,time.time()+1.5, 6, params, direction)

            # alphas = kinematics.computeIK(180,0,-120)
            # alphas = [0,0,0]
            
            # robot.legs[1][0].goal_position = math.degrees(alphas[0])
            # robot.legs[1][1].goal_position = math.degrees(alphas[1])
            # robot.legs[1][2].goal_position = math.degrees(alphas[2])


            # utils.setPositionToLeg(180,0,-120,robot.legs[1])

            utils.setAnglesToLeg(alphas1, robot.legs[1])
            utils.setAnglesToLeg(alphas2, robot.legs[2])
            utils.setAnglesToLeg(alphas3, robot.legs[3])
            utils.setAnglesToLeg(alphas4, robot.legs[4])
            utils.setAnglesToLeg(alphas5, robot.legs[5])
            utils.setAnglesToLeg(alphas6, robot.legs[6])

            # sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))

            if first_move :
                robot.smooth_tick_read_and_write(3)
                first_move = False
            else :
                robot.tick_read_and_write()
            # Ticking the simulation
            sim.tick()
        return
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


main()