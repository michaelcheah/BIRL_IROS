#!/usr/bin/env python
# Scripts for iros challenge 7: store 6 wooden blocks onto correct pegs
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt
import numpy as np
import os

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

import iros_vision_tools as ivt
import iros_vision_functions as ivfunc
PATH_TO_TASK_IMAGES = "task_images"

# Pre defined parameters
peg_log =[[-100, -100],[-100, -110],[-100, -120],[-100, -130],[-100, -140]]
height_pick = 10
height_place = 40

act_objects= [60, 60, 60, 60, 60, 60]

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    for i in range(0,6):
        # Vision - Extract the list of shapes
        task_img_7 = ivt.capture_pic(CAMERA,1)
        cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_7'+str(i)+'.jpg'), task_img_7)
        crop_task_img_4 = ivt.crop_out(task_img_7, crop_points)
        
        shape_list = ivfunc.extract_shape_list(task_img_7, threshold=120, show=True)
        
        if len(shape_list)==0:
            print "No more shapes left"
            print "Expected "+str(5-i)+" more shapes"
            continue
        piece = shape_list[0]
        x1 = piece['point1'][0][0]
        y1 = piece['point1'][0][1]
        x2 = piece['point2'][0][0]
        y2 = piece['point2'][0][1]
        
        params = [piece['shape'], x1, y1, x2, y2]
        
        # Get paramters and put into the following data structure
        # Get paramters and put into the following data structure
        params = [0, x_pos, y_pos, ori]     # where the first item is the number that states the object type: 0 = cirlce, 1 = rect, 2 = triangle etc.

        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_objects[params[0]]
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=4)

        # Go to X,Y centre of the location
        demand_Pose["x"] = params[1]
        demand_Pose["y"] = params[2]
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        ## Set orientation
        demand_Pose = get_ur_position(c,3)
        demand_Pose["rz"] = params[3]
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        # Move down to Grasp
        demand_Pose["z"] = height_pick
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        ## Close Gripper
        demand_Grip["servo"]=0
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        # Pick up object
        demand_Pose["z"] = height_pick +90
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        # Rotate to zero orientation
        demand_joints = get_ur_position(c,3)
        demand_Pose = dict(demand_joints)
        demand_Pose["rz"] = 0
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        # Move to peg holder
        location = peg_log[params[0]]
        demand_Pose["x"] = location[0]
        demand_Pose["y"] = location[1]
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        # Lower
        demand_Pose["z"] = height_place
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        # Release
        demand_Grip["servo"]= 90
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        # Raise from peg
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=4)
