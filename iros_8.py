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

hammer_waypoint_joints = {"x": 1.16, "y": -68.88, "z": 73.40, "rx": -94.51, "ry": -89.69, "rz": 141.78}
hammer_waypoint2_joints = {"x": -18.53, "y": -52.93, "z": 75.96, "rx": 22.47, "ry": -25.38, "rz": 2.46}
nail_waypoint_joints = {"x": -17.84, "y": -78.84, "z": 82.36, "rx": -55.20, "ry": -27.30, "rz": -90.49}

def begin(c,ser_ee):
    hx=-300
    hy=-600
    hz=20
    nx=[-600,-600,-600,-600,-600]
    ny=[-200,-250,-300,-350,-400]
    nz=50
    act_hammer=60

    #motion stuff: pick hammer
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_hammer
    demand_Grip["tilt"]=48
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)    
    
    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)

    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints),CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":hz, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=45
    msg = ic.end_effector_move(ser_ee,Grip=demand_Grip)

    demand_Grip["act"]=80
    msg = ic.end_effector_move(ser_ee,Grip=demand_Grip)

    demand_Grip["servo"]=30
    demand_Grip["tilt"]=58
    msg = ic.end_effector_move(ser_ee,Grip=demand_Grip)

    time.sleep(1)

    demand_Pose["z"]=hz+100
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    msg = ic.safe_move(c,ser_ee,Pose=dict(hammer_waypoint2_joints),Grip=demand_Grip,CMD=2)

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
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":hz+20, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    demand_Grip["servo"]=120
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=current_Pose[2]
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"
