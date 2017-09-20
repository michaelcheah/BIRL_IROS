#!/usr/bin/env python
# Scripts for iros challenge 4: pour water into a cup
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

jug_waypoint_joints = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    act_jug=70
    pour_offset=100

    #vision stuff
    task_img_4 = ivt.capture_pic(CAMERA,1)
    cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_4.jpg'), task_img_4)
    
    crop_task_img_4 = ivt.crop_out(task_img_4, crop_points)
    CAL_PARAM = {'thresh': [75, 100],
                 'radius': [30,45]}
    m_circle, m_cimg = ivt.find_circles(copy.copy(img_4), 3, param=CAL_PARAM, blur=1, show=False)
    plt.imshow(m_cimg)
    plt.show()
    
    mx=[]
    my=[]
    for mug in range(3):
        m_pix = [m_circle[0][mug][0], m_circle[0][mug][1]]
        mx_,my_ = ivt.pix3world(p1, inverse, m_pix)
        mx.append(mx_[0,0])
        my.append(my_[0,0])

    print "MX: ", mx
    print "MY: ", my

    #motion stuff: pick mug
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_mug
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)    
    
    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)

    msg = ic.safe_ur_move(c,Pose=dict(jug_waypoint_joints),CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1]+50, "z":current_Pose[2]-50, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    demand_Grip = dict(iw.ee_home)
    demand_Grip["servo"]=30
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=current_Pose[2]+50
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    for i in range(0,3):
        demand_Pose["x"]=mx[i]+pour_offset
        demand_Pose["y"]=my[i]
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        time.sleep(2)

        full_jug = measure_av_force(c)
        print "average force: ",full_jug

        current_Joints = ic.get_ur_position(c,3)
        demand_Joints = {"x":current_Joints[0], "y":current_Joints[1], "z":current_Joints[2], "rx":current_Joints[3], "ry":current_Joints[4], "rz":current_Joints[5]} 

        j=0
        while fz < full_jug-2 and j<10:
            demand_Joints["rz"]=current_Joints[5]+10*j
            msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.2)

            time.sleep(2)

            fz = measure_av_force(c)
            print "average force: ",fz

        demand_Joints["rz"]=current_Joints[5]
        msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1]+50, "z":current_Pose[2]+50, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=current_Pose[2]-50
    demand_Grip["servo"]=120
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["y"]=current_Pose[1]
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    msg = ic.safe_ur_move(c,Pose=dict(jug_waypoint_joints),CMD=2)

    ic.socket_send(c,sCMD=200)
    msg = ic.safe_ur_move(c,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"

def measure_av_force(c,tolerance=0.01):
    n=0
    fz=0.0
    while True:
        force = ic.get_ur_position(c,6)
        if abs(fz/float(n)-force[2])<tolerance:
            break
        fz = fz+force[2]
        n = n+1
        print "fz: ",fz/float(n)
        print "n: ",n

    return fz/float(n)
