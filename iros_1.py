#!/usr/bin/env python
# Scripts for iros challenge 1: pick up a mug and place on saucer
#                               pick up saucer with mug on
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

saucer_waypoint = {"x": -4.28, "y": -101.02, "z": 122.04, "rx": -103.10, "ry": -24.20, "rz": -54.94}

def begin(c,ser_ee,inverse):
    #object grasping parameters
    act_mug=80
    act_saucer=80
    height_mug=20.0
    height_saucer=20.0
    radius_mug=40.0
    radius_saucer=40.0

    #vision stuff: get mug and saucer position
    # mug and saucer centre positions
    #mx,my,sx,sy = mug_saucer_pos

    mx = float(raw_input("mx: "))
    my = float(raw_input("my: "))
    sx = float(raw_input("sx: "))
    sy = float(raw_input("sy: "))

    #motion stuff: pick mug
    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)
    
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_mug
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    demand_Pose = dict(iw.home)
    demand_Pose["x"]=mx + radius_mug/1.41421
    demand_Pose["y"]=my - radius_mug/1.41421
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_mug
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=0
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    time.sleep(1)

    demand_Pose["z"]=height_mug+height_saucer+40
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["x"]=sx + radius_mug/1.41421
    demand_Pose["y"]=sy - radius_mug/1.41421
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_mug+height_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=80
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    demand_Pose["z"]=120
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    #motion stuff: pick saucer
    demand_Grip["act"]=act_mug
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    demand_Joints = dict(saucer_waypoint)
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":sx, "y":sy+radius_saucer, "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+40
    demand_Grip["servo"]=20
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    time.sleep(0.2)

    demand_Pose["z"]=height_saucer
    demand_Grip["servo"]=0
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    time.sleep(1)

    demand_Pose["z"]=height_saucer+50
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    x=float(raw_input("x: "))
    y=float(raw_input("y: "))

    demand_Pose["x"]=x
    demand_Pose["y"]=y+radius_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer
    demand_Grip["servo"]=20
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=height_saucer+40
    demand_Grip["servo"]=80
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=current_Pose[2]
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)
   
    print ".....................Done......................"
