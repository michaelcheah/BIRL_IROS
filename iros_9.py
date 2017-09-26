#!/usr/bin/env python 
# Scripts for iros challenge 9: pick up a straw 
# insert straw into to go cup 
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

# parameters 
ROTATION=3

act_straw = 75 
heigh_pick = -20

cup_1 = [-400, -400] # Location of cup 1 
cup_2 = [-500, -400]  # Location of cup 2

grasp = {"x": 81.47, "y": -68.40, "z": 129.09, "rx": -219.62, "ry": -131.70, "rz": -26.93} 

way_1 = {"x": 104.95, "y": -91.30, "z": 106.71, "rx": -102.23, "ry": -96.60, "rz": -226.57}
way_2 = {"x": 104.95, "y": -91.30, "z": 106.71, "rx": -102.23, "ry": -96.60, "rz": -226.57}

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):

    x_n, y_n, orient = get_grasping_coords([x1, y1], [x2, y2])

    # Home 
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_straw 
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2) 

    ic.socket_send(c,sCMD=201)

    # Move to straw location
    msg = ic.safe_ur_move(c,Pose=dict(grasp),CMD=2) 

    # Grasp straw 
    demand_Grip["servo"]=30 
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    time.sleep(0.5)

    # Lift straw

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2]+250, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Move to drop straw

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":-200, "y":-400, "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":-200, "y":-400, "z":150, "rx":current_Pose[3], "ry":current_Pose[4]+50, "rz":current_Pose[5]} 
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Drop straw
    demand_Grip["servo"]=100 
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    msg = ic.safe_ur_move(c,Pose=dict(iw.home_joints),CMD=2) 

    ## Set orientation
    current_Joints = ic.get_ur_position(c,3)
    demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]-orient+135}
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    # Go to X,Y centre of the location
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":x1,"y":y1,"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_straw
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    # Move down to Grasp
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose["z"]=height_pick
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)


    ## Close Gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee, demand_Grip)

    time.sleep(0.5)

    # Lift straw
    demand_Pose["z"] = height_pick + 100
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    # Way point 1
    #msg = ic.safe_ur_move(c,Pose=dict(way_1),CMD=2) 

    # Way point 2 
    #msg = ic.safe_ur_move(c,Pose=dict(way_2),CMD=2) 

    # Lower into cup
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":100,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}

    msg = ic.safe_ur_move(c,Pose=demand_Pose,Speed=0.2,CMD=4)

    

def get_grasping_coords(p_centre,p_edge):
    ori = math.atan2(p_centre[1]-p_edge[1],p_centre[0]-p_edge[0])*180.0/math.pi
    print "ori: ",ori
    ori = ori-180
    if ori<-180:
        ori=360+ori
    x = p_edge[0]
    y = p_edge[1]
    return float(x), float(y), ori
    
    
