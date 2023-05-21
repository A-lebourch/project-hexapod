import utils
from utils import SimpleRobotSimulation, Parameters, setPositionToRobot
import pybullet as p
import kinematics
import time
from scipy.spatial.transform import Rotation

def gestion_handle(keys,sim,robot,x=0,y=0,z=0,params=0,w=0,h=0,direction=0,direction2=0,direction3=0):

    forward = direction
    #fonction qui permet de gerrer la gestion du clavier 
    for k,v in keys.items():

                    if (k == p.B3G_RIGHT_ARROW ):
                            forward = 20
                            #droite(x,y,z,h,w,robot,params,direction3)
                            tourner(x,z,h,w,direction,params,robot)

                    if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                            #forward = 0
                            pass
                    if (k == p.B3G_LEFT_ARROW ):
                            forward =  -20
                            #gauche(x,z,h,w,params,robot,direction2)
                            innverse_tournerr(x,z,h,w,direction,params,robot)
                    if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                            #forward = 0
                            pass

                    if (k == p.B3G_UP_ARROW ):
                            forward += 5
                            avancer(x,z,h,w,direction,params,robot)
                           
                    if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                            #forward += 0
                            pass
                    if (k == p.B3G_DOWN_ARROW ):
                            forward=-20
                            reculer(x,z,h,w,direction,params,robot)

                    if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                            #forward=0
                            pass

def avancer(x,z,h,w,direction,params,robot):
    print("J'avance")
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
    print("Je rotate a droite")
    alphas1 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
    alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 3, params, direction)

    utils.setAnglesToLeg(alphas1, robot.legs[1])
    utils.setAnglesToLeg(alphas2, robot.legs[2])
    utils.setAnglesToLeg(alphas1, robot.legs[3])
    utils.setAnglesToLeg(alphas2, robot.legs[4])
    utils.setAnglesToLeg(alphas1, robot.legs[5])
    utils.setAnglesToLeg(alphas2, robot.legs[6])

def innverse_tournerr(x,z,h,w,direction,params,robot):

    print('Je rotate à gauche')
    alphas1 = kinematics.triangle(x,z,h,w,time.time(), 3, params, -direction)
    alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 3, params, -direction)

    utils.setAnglesToLeg(alphas1, robot.legs[1])
    utils.setAnglesToLeg(alphas2, robot.legs[2])
    utils.setAnglesToLeg(alphas1, robot.legs[3])
    utils.setAnglesToLeg(alphas2, robot.legs[4])
    utils.setAnglesToLeg(alphas1, robot.legs[5])
    utils.setAnglesToLeg(alphas2, robot.legs[6])

def reculer(x,z,h,w,direction,params,robot):
    print("Je recule")
    # la je vien mettre en place les éléments afin de charger le mode de 
    
    alphas1 = kinematics.triangle(x,z,h,w,time.time(), 1, params, -direction)
    alphas2 = kinematics.triangle(-x,z,h,w,time.time()+1.5, 2, params, -direction)
    alphas3 = kinematics.triangle(-x,z,h,w,time.time(), 3, params, -direction)
    alphas4 = kinematics.triangle(-x,z,h,w,time.time()+1.5, 4, params, -direction)
    alphas5 = kinematics.triangle(x,z,h,w,time.time(), 5, params, -direction)
    alphas6 = kinematics.triangle(x,z,h,w,time.time()+1.5, 6, params, -direction)

    utils.setAnglesToLeg(alphas1, robot.legs[1])
    utils.setAnglesToLeg(alphas2, robot.legs[2])
    utils.setAnglesToLeg(alphas3, robot.legs[3])
    utils.setAnglesToLeg(alphas4, robot.legs[4])
    utils.setAnglesToLeg(alphas5, robot.legs[5])
    utils.setAnglesToLeg(alphas6, robot.legs[6])

def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat