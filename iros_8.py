#!/usr/bin/env python
# Scripts for iros challenge 8: pick up a hammer
#                               use this to drive 5 nails into foam board
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

nail_waypoint_joints = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}

def begin(c,ser_ee):
    hx=-300
    hy=-600
    hz=30
    nx=[-600,-600,-600,-600,-600]
    ny=[-200,-250,-300,-350,-400]
    nz=30
    act_hammer=70

    #motion stuff: pick mug
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_mug
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)    
    
    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)

    current_Joints = ic.get_ur_position(c,CMD=3)
    demand_Joints = {"x":current_Joints[0], "y":current_Joints[1], "z":current_Joints[2], "rx":current_Joints[3], "ry":current_Joints[4], "rz":current_Joints[5]+45} 
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":hx, "y":hy, "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=hz
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=0
    msg = ic.end_effector_move(ser_ee,Grip=demand_Grip)

    demand_Pose["z"]=hz+100
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    msg = ic.safe_ur_move(c,Pose=dict(nail_waypoint_joints),CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    for i in range(0,5):
        demand_Pose = {"x":nx[i], "y":ny[i], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        for j in range(0,2):
            demand_Pose["z"]=nz+100
            msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

            demand_Pose["z"]=nz
            msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        demand_Pose["z"]=current_Pose[2]
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        time.sleep(0.5)

    msg = ic.safe_ur_move(c,Pose=dict(nail_waypoint_joints),CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":hx+20, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    demand_Grip["servo"]=80
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=current_Pose[2]
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"