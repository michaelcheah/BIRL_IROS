#!/usr/bin/env python
# Scripts for iros challenge 2: lay out silverware
#                               re-stow silverware
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

#Joint positions for the picking ( just above)
pick_f = {"x": 77.82, "y": -86.76, "z": 105.13, "rx": -102.76, "ry": -92.04, "rz":-58.57}
pick_k = {"x": 71.52, "y": -84.93, "z": 112.03, "rx": -111.79, "ry": -92.66, "rz":-64.81}
pick_s = {"x": 85.44, "y": -85.50, "z": 105.73, "rx": -104.36, "ry": -91.28, "rz": -50.97}


#Joint positions for the placing
pick_f = {"x": 84.25, "y": -73.27, "z": 141.24, "rx": -155.85, "ry": -0.83, "rz": -5.47}
pick_k = {"x": 84.25, "y": -73.27, "z": 141.24, "rx": -155.85, "ry": -0.83, "rz": -5.47}
pick_s = {"x": 22.63, "y": -77.99, "z": 95.92, "rx": -109.28, "ry": -90.67, "rz": -189.17}

pick_height = 4.75
place_height = 4.75

act_f = 75
act_k = 75
act_s = 75

def begin(c,ser_ee):
    # pick up fork
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_f
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Go to just above fork place_height
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_f),CMD=2)

    #Go down in the z to pick up the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = pick_height

    # Close the gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_f),CMD=2)

    # Go to place the fork
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_f),CMD=2)

    # Lower the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = place_height

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise arm
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_f),CMD=2)
    test = raw_input("Go on to spoon?")
###############################################################################
    # pick up spoon
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_s
    msg = ic.safe_move(c,ser_ee,Grip=demand_Grip,CMD=2)

    # Go to just above spoon place_height
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_s),CMD=2)

    #Go down in the z to pick up the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = pick_height

    # Close the gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_s),CMD=2)

    # Go to place the fork
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_s),CMD=2)

    # Lower the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = place_height

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise arm
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_s),CMD=2)
    test = raw_input("Go on to knife?")
###############################################################################
    # pick up knife
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_k
    msg = ic.safe_move(c,ser_ee,Grip=demand_Grip,CMD=2)

    # Go to just above spoon place_height
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_k),CMD=2)

    #Go down in the z to pick up the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = pick_height

    # Close the gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_k),CMD=2)

    # Go to place the fork
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_k),CMD=2)

    # Lower the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = place_height

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise arm
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_k),CMD=2)
    test = raw_input("Go on to retrieve fork?")
###############################################################################
    # pick up fork
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_f
    msg = ic.safe_move(c,ser_ee,Grip=demand_Grip,CMD=2)

    # Go abouve place of fork
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_k),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = place_height

    # Pick up fork
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Lift and move above the tray for fokr
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_k),CMD=2)
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_k),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = pick_height

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_k),CMD=2)

    test = raw_input("Go on to retrieve spoon?")
###############################################################################
    # pick up spoon
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_s
    msg = ic.safe_move(c,ser_ee,Grip=demand_Grip)

    # Go abouve place of fork
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_s),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = place_height

    # Pick up fork
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Lift and move above the tray for fokr
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_s),CMD=2)
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_s),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = pick_height

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_s),CMD=2)

    test = raw_input("Go on to retrieve knife?")
###############################################################################
    # pick up knife
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_k
    msg = ic.safe_move(c,ser_ee,Grip=demand_Grip)

    # Go abouve place of knife
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_k),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = place_height

    # Pick up fork
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Lift and move above the tray for fokr
    msg = ic.safe_move(c,ser_ee,Pose=dict(place_k),CMD=2)
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_k),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = dict(current_Pose)
    demand_Pose["z"] = pick_height

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_k),CMD=2)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"
