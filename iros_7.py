#!/usr/bin/env python
# Scripts for iros challenge 7: store 6 wooden blocks onto correct pegs
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

# Pre defined parameters
peg_log =[[-100, -100],[-100, -110],[-100, -120],[-100, -130],[-100, -140]]
height_pick = 10
height_place = 40

act_objects= [60, 60, 60, 60, 60, 60]



def begin(c,ser_ee):
    for i in range(0,7):
        # Get paramters and put into the following data structure
        params = [0, x_pos, y_pos, ori]

        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_objects[params[0]]
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Go to X,Y centre of the location
        demand_Pose["x"] = params[1]
        demand_Pose["y"] = params[2]
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        ## Set orientation
        demand_Pose = get_ur_position(c,3)
        demand_Pose["rz"] = params[3]
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        # Move down to Grasp
        demand_Pose["z"] = height_pick
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        ## Close Gripper
        demand_Grip["servo"]=0
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        # Pick up object
        demand_Pose["z"] = height_pick +90
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        # Rotate to zero orientation
        demand_Pose = get_ur_position(c,3)
        demand_Pose["rz"] = 0
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        # Move to peg holder
        location = peg_log[params[0]]
        demand_Pose["x"] = location[0]
        demand_Pose["y"] = location[1]
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        # Lower
        demand_Pose["z"] = height_place
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=2)

        # Release
        demand_Grip["servo"]= 90
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        # Raise from peg
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)
