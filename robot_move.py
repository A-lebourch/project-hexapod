import utils
from utils import SimpleRobotSimulation, Parameters, setPositionToRobot
import pybullet as p
import kinematics
import time
from scipy.spatial.transform import Rotation

def gestion_handle(keys,robot,x,z,h,w,params,direction, mode,test=[0,0]):
    for k,v in keys.items():

        if (k == p.B3G_RIGHT_ARROW ):
            if mode == "arrow":
                tourner(x,z,h,w,direction,params,robot)
            if mode == "body":   
                test[1] -= 0.5
        
        if (k == p.B3G_LEFT_ARROW ):
            if mode == "arrow":
                tourner(x,z,h,w,-direction,params,robot)
            if mode == "body":   
                test[1] += 0.5

        if (k == p.B3G_UP_ARROW ):
            if mode == "arrow":
                avancer(x,z,h,w,direction,params,robot)
            if mode == "body":   
                test[0] += 0.5
                
        if (k == p.B3G_DOWN_ARROW ):
            if mode == "arrow":
                avancer(x,z,h,w,-direction,params,robot)
            if mode == "body":   
                test[0] -= 0.5
        
        if k == ord('m'):
            print('youpi')
        
    return test

def avancer(x,z,h,w,direction,params,robot):
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

def tourner(x,z,h,w,direction,params,robot):
    alphas1 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
    alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 3, params, direction)

    utils.setAnglesToLeg(alphas1, robot.legs[1])
    utils.setAnglesToLeg(alphas2, robot.legs[2])
    utils.setAnglesToLeg(alphas1, robot.legs[3])
    utils.setAnglesToLeg(alphas2, robot.legs[4])
    utils.setAnglesToLeg(alphas1, robot.legs[5])
    utils.setAnglesToLeg(alphas2, robot.legs[6])

def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    rot_quat = rot.as_quat()
    return rot_quat