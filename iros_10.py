#!/usr/bin/env python
# Scripts for iros challenge 10: open a bottle with a safety locking cap
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc


def begin(c,ser_ee):

    # parameters
    bottle_radius = 15
    bottle_height = 100

    # Parameters to pass to the function
    bx_1 = 500
    by_1 = 400

    demand_Grip = dict(ee_home)
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.grab_joints),Grip=demand_Grip,CMD=2)

    # Goto XY position for the bottle
    demand_Pose = dict(home)
    demand_Pose["x"] = bx_1 + bottle_radius
    demand_Pose["y"] = by_1
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Lower to bottle
    demand_Pose["z"] = bottle_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Grasp bottle
    demand_Grip["servo"]=10
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Push Lid in
    demand_Pose["z"] = bottle_height - 10
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Twist
    current_Pose = get_ur_position(c,CMD=1)
    # pose_cur = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    new_z = current_Pose[5] + 180   # Rotate in Z through 180 degrees
    Pose_rot = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":new_z}
    msg = ic.safe_ur_move(c,Pose=Pose_rot,CMD=4)

    # Lift up
    demand_Pose["z"] = bottle_height + 100
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)
