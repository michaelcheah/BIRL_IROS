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

hammer_waypoint_joints_1 = {"x": 86.74, "y": -44.87, "z": 36.82, "rx": 5.5, "ry": -6.23, "rz": 151.96}
hammer_waypoint_joints_2 = {"x": 86.73, "y": -36.34, "z": 64.23, "rx": -30.99, "ry": -6.06, "rz": -151.40}
hammer_waypoint_joints_3 = {"x": 79.10, "y": -36.19, "z": 64.40, "rx": -29.64, "ry": -13.68, "rz": -153.14}


hammer_waypoint_joints_4 = {"x": 79.10, "y": -36.19, "z": 64.40, "rx": -29.64, "ry": -13.68, "rz": -153.14}


nail_x = 20
nail_down = 50

def begin(c,ser_ee):
    # Home
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Move towards hammer using waypoints
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_1),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_2),CMD=2)
    time.sleep(1)
    raw = raw_input("continue?")
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_3),CMD=8)

    # Close hammer servo
    ic.serial_send(ser_ee,"H",100)
    time.sleep(2)
    raw = raw_input("continue?")

    # Move to above first nails
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_4),CMD=2, Speed = 0.1)

    # Move along

    for i in range (0, 15):
        updown()
        time.sleep(0.5)
        # Move along in x direction
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        demand_Pose["x"]= demand_Pose["x"] + nail_x

    # Release hammer
    ic.serial_send(ser_ee,"H",100)

    # GO HOme
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"


def updown():
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    demand_Pose["z"]= demand_Pose["z"] - nail_down
    msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD =4)

    time.sleep(1)

    demand_Pose["z"]= demand_Pose["z"] + nail_down
    msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD =4)
