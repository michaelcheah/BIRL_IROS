#!/usr/bin/env python
# Scripts for iros challenge 8: pick up a hammer
#                               use this to drive 5 nails into foam board
# ic.serial_send(ser_ee,"H",var)  0-127
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

hammer_waypoint_joints_1 = {"x": 1.16, "y": -68.88, "z": 73.40, "rx": -94.51, "ry": -89.69, "rz": 141.78}
hammer_waypoint_joints_2 = {"x": 1.16, "y": -68.88, "z": 73.40, "rx": -94.51, "ry": -89.69, "rz": 141.78}
hammer_waypoint_joints_3 = {"x": -18.53, "y": -52.93, "z": 75.96, "rx": 22.47, "ry": -25.38, "rz": 2.46}
hammer_waypoint_joints_4 = {"x": -18.53, "y": -52.93, "z": 75.96, "rx": 22.47, "ry": -25.38, "rz": 2.46}

nail = [0, -300]
nail_up = 80
nail_down = 50

def begin(c,ser_ee):
    # Home
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Move towards hammer using waypoints
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_1),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_2),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_3),CMD=2)

    # Move slowly towards the hammer
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_4),CMD=2, Speed = 0.1)

    # Close hammer servo
    ic.serial_send(ser_ee,"H",100)

    # Pick up hammer (waypoint)
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_5),CMD=2, Speed = 0.1)

    # Move to the first nail_waypoint_joints
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_5),CMD=2, Speed = 0.1)

    # Now move to carterisan space and move along hitting nails
	current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":nail[0],"y":nail[1],"z":nail_up,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD =4)
    # Move along in the x, hitting down as we go
    for i in range (0, 15):
        # move
        demand_Pose["x"] = demand_Pose["x"] + i*5
        msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD = 4)
        #down
        demand_Pose["z"]= nail_down
        msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD = 4)
        time.sleep(0.5)
        #up
        demand_Pose["z"]= nail_up
        msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD = 4)
    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"
