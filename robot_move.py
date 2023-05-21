from utils import *
import pybullet as p
import kinematics
import time
from scipy.spatial.transform import Rotation
import math
import pygame
from pygame.locals import *

def increment_decrement(min_value, max_value, sens, X):
    if sens == "inc":
        X += 1
        if X > max_value:
            X = max_value
            sens = "dec"
    elif sens == "dec":
        X -= 1
        if X < min_value:
            X = min_value
            sens = "inc"
    return X, sens

def DILLIDGAF(window, image, image_rect,robot, mouse_down, mouse_pos):
    for event in pygame.event.get():
        if event.type == MOUSEBUTTONDOWN and event.button == 1: 
            mouse_down = True
        elif event.type == MOUSEBUTTONUP and event.button == 1: 
            mouse_down = False

    if mouse_down:
        mouse_pos = pygame.mouse.get_pos()
        print("Position du curseur :", mouse_pos[0]-397,mouse_pos[1]-293)

    setPositionToLeg(200,mouse_pos[0]-397,-(mouse_pos[1]-293),robot.legs[1])
    return mouse_down, mouse_pos


def speed(keys, gear):
    for k,v in keys.items():
        if k == ord('a'):
            gear = 1

        if k == ord('z'):
            gear = 2

        if k == ord('e'):
            gear = 3

        if k == ord('r'):
            gear = 4
    
    if gear == 1:
        x=0
        z=0
        w=10
        h=30
        return x,z,h,w,gear

    if gear == 2:
        x=0
        z=0
        w=60
        h=30
        return x,z,h,w,gear
    
    if gear == 3:
        x=0
        z=0
        w=120
        h=30
        return x,z,h,w,gear
    
    if gear == 4:
        x=0
        z=0
        w=180
        h=30
        return x,z,h,w,gear

def test(robot,x,z,h,w,params,direction, up_pressed):
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                up_pressed = True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                up_pressed = False

    if up_pressed:
        avancer(x,z,h,w,direction,params,robot)

    time.sleep(0.1)  # Adjust sleep time if needed for responsiveness

    if pygame.key.get_pressed()[pygame.K_DOWN]:
        avancer(x,z,h,w,-direction,params,robot)

    if pygame.key.get_pressed()[pygame.K_RIGHT]:
        tourner(x,z,h,w,direction,params,robot)

    if pygame.key.get_pressed()[pygame.K_LEFT]:
        tourner(x,z,h,w,-direction,params,robot)
    return up_pressed
    
def button_press(keys,robot,x,z,h,w,params,direction, mode,test=[0,0]):
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
        
        if k == ord('l'):
            if mode == "arrow":
                avancer(x,z,h,w,direction-math.pi/2,params,robot)
        
        if k == ord('m'):
            if mode == "arrow":
                avancer(x,z,h,w,direction+math.pi/2,params,robot)

    return test

def avancer(x,z,h,w,direction,params,robot):
    alphas1 = kinematics.triangle(-x,z,h,w,time.time(), 1, params, direction)
    alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 2, params, direction)
    alphas3 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
    alphas4 = kinematics.triangle(x,z,h,w,time.time()+1.5, 4, params, direction)
    alphas5 = kinematics.triangle(-x,z,h,w,time.time(), 5, params, direction)
    alphas6 = kinematics.triangle(-x,z,h,w,time.time()+1.5, 6, params, direction)

    setAnglesToLeg(alphas1, robot.legs[1])
    setAnglesToLeg(alphas2, robot.legs[2])
    setAnglesToLeg(alphas3, robot.legs[3])
    setAnglesToLeg(alphas4, robot.legs[4])
    setAnglesToLeg(alphas5, robot.legs[5])
    setAnglesToLeg(alphas6, robot.legs[6])

def tourner(x,z,h,w,direction,params,robot):
    alphas1 = kinematics.triangle(x,z,h,w,time.time(), 3, params, direction)
    alphas2 = kinematics.triangle(x,z,h,w,time.time()+1.5, 3, params, direction)

    setAnglesToLeg(alphas1, robot.legs[1])
    setAnglesToLeg(alphas2, robot.legs[2])
    setAnglesToLeg(alphas1, robot.legs[3])
    setAnglesToLeg(alphas2, robot.legs[4])
    setAnglesToLeg(alphas1, robot.legs[5])
    setAnglesToLeg(alphas2, robot.legs[6])

def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    rot_quat = rot.as_quat()
    return rot_quat