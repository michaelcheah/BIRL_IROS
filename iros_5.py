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

# Pre Determined stuff
act_usb=72
pos_u = [-400, -400]            # Location to go to for the USB
height_u = 80

pos_light = [-450, -400]        #
height_light = 60               #
act_light= 20

usb1_down = {"x":65.87,"y":-105.16,"z":137.25,"rx":-132.26,"ry":-118.31,"rz":39.85}
usb2_down = {"x":67.64,"y":-104.63,"z":135.12,"rx":-129.68,"ry":-118.59,"rz":41.82}
usb3_down = {"x":68.78,"y":-102.54,"z":133.46,"rx":-129.50,"ry":-118.78,"rz":43.11}
usb4_down = {"x":69.70,"y":-100.06,"z":131.46,"rx":-129.46,"ry":-118.91,"rz":44.15}
usb_joints_waypoint = {"x":-44.27,"y":-96.15,"z":104.97,"rx":-34.63,"ry":-46.86,"rz":-207.27}

mains_waypoint_joints = {"x":60.64,"y":-108.07,"z":126.11,"rx":-108.03,"ry":-89.70,"rz":-81.74}

PLUG_REL_CROP = [120, -150, -46, -19]
PLUG_POINTS = [0, 22, 44, 66]
PLUG_SPACE = 24
PLUG_THRESH=-90

USB_REL_CROP = [217, -95, -46, -19]
USB_POINTS = [0, 10, 22, 32]
USB_SPACE = 12
USB_THRESH = 130
ROTATION = 3

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    while True:
        vision_check = raw_input("Vision? (yes/no)")
        if vision_check == "no":
            break
        elif vision_check == "yes":
            task_img_5 = ivt.capture_pic(CAMERA,ROTATION)
            cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_5.jpg'), task_img_5)
            answer = ivfunc.find_usb(task_img_5, crop_points, params = [USB_REL_CROP, USB_POINTS, USB_SPACE, USB_THRESH])
            print "ANSWER:  ", answer
            answer2 = ivfunc.find_usb(task_img_5, crop_points, params = [PLUG_REL_CROP, PLUG_POINTS, PLUG_SPACE, PLUG_THRESH])
            print "ANSWER2: ", answer2

    obj = raw_input('Light or USB? (l/u)?')
    if (obj == "u"):
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_usb
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Set tool to iros_1
        ic.socket_send(c,sCMD=201)

        # Close Gripper
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        inp =raw_input("give USB")
	if (inp =="1"):
            msg = ic.safe_ur_move(c,Pose=dict(usb1_down),CMD=2, Speed = 0.2)
        if (inp =="2"):
            msg = ic.safe_ur_move(c,Pose=dict(usb2_down),CMD=2, Speed = 0.2)
	if (inp =="3"):
            msg = ic.safe_ur_move(c,Pose=dict(usb3_down),CMD=2, Speed = 0.2)
        if (inp =="4"):
            msg = ic.safe_ur_move(c,Pose=dict(usb4_down),CMD=2, Speed = 0.2)

        # Close Gripper
        demand_Grip["servo"]=18
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)

        raw= raw_input("wait")

        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        demand_Pose["z"] = demand_Pose["z"] + 45
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,Speed=0.2,CMD=4)	
        
        time.sleep(2)

        # Open Gripper
        demand_Grip["servo"]=120
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)


    '''else:
        # Home position
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_light
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Set tool to iros_1
        ic.socket_send(c,sCMD=201)

        # Goto position of light
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":pos_light[0], "y":pos_light[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Lower to light
        demand_Pose["z"] = height_light
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Close Gripper
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)

        # Pull out
        demand_Pose["z"] =  height_light + 50
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Push in and release
        demand_Pose["z"] =  height_light  - 5
        demand_Grip["servo"]=120
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Speed=0.25,Grip=demand_Grip,CMD=4)'''

    if (obj == "l"):
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=47
        msg = ic.safe_move(c,ser_ee,Pose=dict(mains_waypoint_joints),Grip=demand_Grip,CMD=2)

        plug = int(raw_input("plug: "))

        # Goto position of light
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0], "y":current_Pose[1]-28.7*(plug-1), "z":current_Pose[2]-45, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        demand_Pose["x"]=current_Pose[0]-25
        demand_Grip["servo"]=37
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        time.sleep(0.5)

        demand_Pose["z"]=current_Pose[2]+20
        msg = ic.safe_ur_move(c,Pose=demand_Pose,Speed=0.05,CMD=4)

        time.sleep(2)

        demand_Pose["z"]=current_Pose[2]-45
        demand_Grip["servo"]=120
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,Speed=0.05,CMD=4)

        time.sleep(0.5)
        

    # Raise
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2]+80, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Set tool to iros_0
    ic.socket_send(c,sCMD=200)

    # Home position
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"        
