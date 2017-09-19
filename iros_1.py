#!/usr/bin/env python
# Scripts for iros challenge 1: pick up a mug and place on saucer
#                               pick up saucer with mug on
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt
import os

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

import iros_vision_tools as ivt
import iros_vision_functions as ivfunc

PATH_TO_TASK_IMAGES = "task_images"

saucer_waypoint = {"x": -4.28, "y": -101.02, "z": 122.04, "rx": -103.10, "ry": -24.20, "rz": -54.94}

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    #object grasping parameters
    act_mug=80
    act_saucer=80
    height_mug=20.0
    height_saucer=5.0
    radius_mug=43.0
    radius_saucer=45.0
    
    task_img_1 = ivt.capture_pic(CAMERA,1)
    cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_1.jpg'), task_img_1)
    crop_task_img_1 = ivt.crop_out(task_img_1, crop_points)
    table_circles = ivfunc.cup_saucer(crop_task_img_1, show=True)
    print "CROP_POINTS: ", crop_points
    print "P1: ", p1
    print "INVERSE", inverse

    m_circle = table_circles["mug"]["circle"]
    s_circle = table_circles["saucer"]["circle"]
    print "m_circle: ", m_circle
    print "s_circle: ", s_circle
    mp = [m_circle[0], m_circle[1]]
    mx,my = ivt.pix3world(p1, inverse, mp)
    mx = mx[0,0]
    my = my[0,0]
    
    sp = [s_circle[0], s_circle[1]]
    sx,sy = ivt.pix3world(p1, inverse, sp)
    sx = sx[0,0]
    sy = sy[0,0]
    
    #vision stuff: get mug and saucer position
    # mug and saucer centre positions
    #mx,my,sx,sy = mug_saucer_pos

    print "MX: ", mx
    print "MY: ", my
    
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=60
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)
    
    demand_Pose = dict(iw.home)
    demand_Pose["x"]=mx + radius_mug/1.41421
    demand_Pose["y"]=my - radius_mug/1.41421
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_mug
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=10
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    
    demand_Grip["act"]=act_mug
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    
    demand_Grip["servo"]=50
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    
    demand_Grip["servo"]=0
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    time.sleep(1)

    demand_Pose["z"]=height_mug+height_saucer+40
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["x"]=sx + radius_mug/1.41421
    demand_Pose["y"]=sy - radius_mug/1.41421
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_mug+height_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=80
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    demand_Pose["z"]=120
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    #motion stuff: pick saucer
    demand_Grip["act"]=act_saucer
    demand_Grip["tilt"]=0
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    demand_Joints = dict(saucer_waypoint)
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":sx, "y":sy+radius_saucer, "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+40
    demand_Grip["servo"]=30
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    time.sleep(0.2)

    demand_Pose["z"]=height_saucer
    demand_Grip["servo"]=0
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    time.sleep(1)

    demand_Pose["z"]=height_saucer+50
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    x=float(raw_input("x: "))
    y=float(raw_input("y: "))

    demand_Pose["x"]=x
    demand_Pose["y"]=y+radius_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer
    demand_Grip["servo"]=20
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=height_saucer+40
    demand_Grip["servo"]=80
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=current_Pose[2]
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)
   
    print ".....................Done......................"
