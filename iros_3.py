#!/usr/bin/env python
# Scripts for iros challenge 3: stir a mug of water with a spoon

import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc


## Way points
ee_home = {"act": 80, "servo": 80, "tilt": 30, "vac": "r"}
home = {"x": 90.0, "y": -500.0, "z": 100.0, "rx": 0.0, "ry": 180.0, "rz": 0.0}
home_joints = {"x": 87.61, "y": -87.40, "z": 100.79, "rx": -103.37, "ry": -89.70, "rz": -2.26}


## Object parameters
cup_radius = 25
cup_height = 40
spoon_bowl = 15         # lenght of spoon bowl (to be convered when stirring)
stir_radius = cup_radius - 10

## Location parameters
mx_1 = 100
my_1  = 200
attack_angle =70

## Paremters to be passed
mx_2 = 200
my_2 = 300

#ic.socket_send(c,sCMD=202)
    def begin(c,ser_ee):
            # Home for end effector and actuator
        demand_Grip = dict(ee_home)
        demand_Grip = act_spoon
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.grab_joints),Grip=demand_Grip,CMD=2)

        # Goto XY position for the spoon
        demand_Pose = dict(home)
        demand_Pose["x"]=mx_1
        demand_Pose["y"]=my_1
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Goto correct orientation
        get_grasping_coords(p_centre,p_edge,height)

        # Grasp spoon
        demand_Grip["servo"]=0
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        # Lift spoon
        demand_Pose["z"]=cup_height + 20
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        ## Move to second cup x, y
        demand_Pose = dict(home)
        demand_Pose["x"]=mx_2
        demand_Pose["y"]=my_2
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        ## Lower spoon
        demand_Pose["z"]=cup_height-spoon_bowl
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        ## Stir spoon
        add_stir = [0, stir_radius, 0, -stir_radius, 0]
        for j in range (0,1):
            for i in range (0,3):
                demand_Pose["x"]=mx_2 + add_stir[i+1]
                demand_Pose["y"]=my_2 + add_stir[i]
                msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        ## Lift spoon
        demand_Pose["z"]=cup_height+20
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4

    ## Home
        demand_Pose = dict(home)
        demand_Pose["x"]=mx_1
        demand_Pose["y"]=my_1
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)





def get_grasping_coords(p_centre,p_edge,height):
    aoa = attack_angle
    ori = math.atan2(p_centre[1]-p_edge[1],p_centre[0]-p_edge[0])*180.0/math.pi
    size = math.sqrt(math.pow(p_centre[0]-p_edge[0],2)+math.pow(p_centre[1]-p_edge[1],2))
    print "ori: ",ori
    ori = ori-180
    if ori<-180:
        ori=360+ori
    x = p_edge[0]
    y = p_edge[1]
    z = height
    return float(x[0]), float(y[0]), z, ori, aoa, size
