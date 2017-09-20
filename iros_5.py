import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

# Pre Determined stuff
act_usb=70
pos_u = [-400, -400]            # Location to go to for the USB
height_u = 80

pos_light = [-450, -400]        #
height_light = 60               #
act_light= 50


def begin(c,ser_ee):
    obj = raw_input('Light or USB? (l/u)?')
    if (obj == "u"):
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_usb
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Go to waypoint to get correct orientation or the joints to come in from the side to pick
        demand_Pose = dict(iw.grabbing_joints_waypoint)
        demand_Pose["x"]=pos_u[0]
        demand_Pose["z"]=height_u
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Move forwards to the USB light
        demand_Pose = dict(home)
        demand_Pose["y"] =pos_u[1]
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Close Gripper
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        #Pull out
        demand_Pose["z"] = height_u + 40
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        time.sleep(5)

        #Push in
        demand_Pose["z"] = height_u - 5
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Release Gripper
        demand_Grip["servo"]=120
        msg = ic.end_effector_move(ser_ee,demand_Grip)

    else:
        # Home position
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_light
        demandPose =  dict(iw.home_joints)
        msg = ic.safe_move(c,ser_ee,Pose=demandPose,Grip=demand_Grip,CMD=2)

        # Goto position of light
        demand_Pose["x"] = pos_light[0]
        demand_Pose["y"] = pos_light[1]
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Lower to light
        demand_Pose["z"] = height_light
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Close Gripper
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        # Pull out
        demand_Pose["z"] =  height_light + 40
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Push in
        demand_Pose["z"] =  height_light  - 5
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Release Gripper
        demand_Grip["servo"]=120
        msg = ic.end_effector_move(ser_ee,demand_Grip)
