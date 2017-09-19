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
    act_bottle = 60

    # Parameters to pass to the function
    bx_1 = -400
    by_1 = -400

    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_bottle
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Goto XY position for the bottle
    demand_Pose = dict(iw.home)
    demand_Pose["x"] = bx_1 + bottle_radius/1.414
    demand_Pose["y"] = by_1 - bottle_radius/1.414
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Lower to bottle
    demand_Pose["z"] = bottle_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Grasp bottle
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Push Lid in
    demand_Pose["z"] = bottle_height - 10
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Twist
    current_Joints = get_ur_position(c,CMD=3)
    demand_Joints = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]+180}
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    # Lift up
    demand_Pose["z"] = bottle_height + 100
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)
