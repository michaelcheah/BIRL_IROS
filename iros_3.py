#!/usr/bin/env python
# Scripts for iros challenge 3: stir a mug of water with a spoon

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

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    ## Object parameters
    cup_radius = 40
    cup_height = 60
    spoon_bowl = 0         # lenght of spoon bowl (to be convered when stirring)
    spoon_height = 20
    stir_radius = cup_radius - 20
    act_spoon = 75
    
    task_img_3 = ivt.capture_pic(CAMERA,1)
    cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_3.jpg'), task_img_3)
    
    crop_task_img_3 = ivt.crop_out(task_img_3, crop_points)
    
    img_3a = crop_task_img_3[0:180, 0:-1]
    img_3b = ivt.black_out(crop_task_img_3, [180,-1,0,-1])
    
    p_circle, spoon = ivfunc.find_spoon(img_3a, show=True)
    
    #vision stuff: get mug and saucer position
    # mug and saucer centre positions
    #mx,my,sx,sy = mug_saucer_pos
    
    ## Location of first mug ()
    p_pix = [p_circle[0][0],p_circle[0][1]]
    print "P_PIX: ", p_pix
    px,py = ivt.pix3world(p1, inverse, p_pix)
    px = px[0,0]
    py = py[0,0]
    
    print "SPOON: ", spoon
    s_pix = [spoon[0][0], spoon[0][1]]
    print "S_PIX: ", s_pix
    sx,sy = ivt.pix3world(p1, inverse, s_pix)
    sx = sx[0,0]
    sy = sy[0,0]
    print "PX, PY, SX, SY: ", px, py, sx, sy
    
    p_centre = [px, py]
    p_edge = [sx, sy]
    attack_angle=70
    print "P_CENTRE: ", p_centre
    print "P_EDGE:   ", p_edge
    
    ## Location of Second Mug
    CAL_PARAM = {'thresh': [75, 100],
                 'radius': [30,45]}
    m_circle, m_cimg = ivt.find_circles(copy.copy(img_3b), 1, param=CAL_PARAM, blur=1, show=False)
    plt.imshow(m_cimg)
    plt.show()
    m_pix = [m_circle[0][0][0],m_circle[0][0][1]]
    mx,my = ivt.pix3world(p1, inverse, m_pix)
    mx_2 = mx[0,0]
    my_2 = my[0,0]
    print "MX, MY: ", mx, my

    # Home for end effector and actuator
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"] = act_spoon
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    ic.socket_send(c,sCMD=201)

    # Home for end effector and actuator
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"] = act_spoon
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    ic.socket_send(c,sCMD=201)

    # Goto spoon (TO FINISH)
    x_p, y_p, ori = get_grasping_coords(p_edge,p_centre)
    x_p = p_edge[0]
    y_p = p_edge[1]
    ori = ori+90
    angle_grasp(c,ser_ee,ori,attack_angle)

    current_Joints = ic.get_ur_position(c,3)
    demand_Joints = {"x":current_Joints[0], "y":current_Joints[1], "z":current_Joints[2], "rx":current_Joints[3], "ry":current_Joints[4], "rz":current_Joints[5]+90}
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":cup_height+spoon_height, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]} 
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["x"]=x_p
    demand_Pose["y"]=y_p
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Grasp spoon
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Lift spoon
    demand_Pose["z"]=cup_height+spoon_height+60
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ## Move to second cup x, y
    demand_Pose = dict(iw.home)
    demand_Pose["x"]=mx_2
    demand_Pose["y"]=my_2
    demand_Pose["z"]=cup_height+spoon_height+60
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ## Lower spoon
    demand_Pose["z"]=cup_height+spoon_height-spoon_bowl
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ## Stir spoon
    add_stir = [0, stir_radius, 0, -stir_radius, 0]
    for j in range (0,2):
        for i in range (0,4):
            demand_Pose["x"]=mx_2 + add_stir[i+1]
            demand_Pose["y"]=my_2 + add_stir[i]
            msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ## Lift spoon
    demand_Pose["z"]=cup_height+spoon_height+60
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ic.socket_send(c,sCMD=200)

    ## Home
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"

def get_grasping_coords(p_centre,p_edge):
    #aoa = 70
    ori = math.atan2(p_centre[1]-p_edge[1],p_centre[0]-p_edge[0])*180.0/math.pi
    print "ori: ",ori
    ori = ori-180
    if ori<-180:
        ori=360+ori
    x = p_edge[0]
    y = p_edge[1]
    return float(x), float(y), ori


def angle_grasp(c,ser_ee,orientation,angle_of_attack):
    # Break-up rotations into max 90degrees
    thetaz = 0
    if orientation>90:
        orientation=orientation-90
        thetaz=math.pi/2
    elif orientation<-90:
        orientation=orientation+90
        thetaz=-math.pi/2

    # Avoid singularity at +/-45degrees
    if orientation==45:
        orientation = 44
    elif orientation==-45:
        orientation = -44

    # Convert to radians
    angle_of_attack=angle_of_attack*math.pi/180.0
    orientation=orientation*math.pi/180.0
    thetay=135.0*math.pi/180.0

    # Cartesian rotation matrices to match uw.grabbing_joints rotation
    x_rot = np.matrix([[ 1.0, 0.0, 0.0],
             [ 0.0, math.cos(math.pi/2), -math.sin(math.pi/2)],
             [ 0.0, math.sin(math.pi/2), math.cos(math.pi/2)]]) # x_rot[rows][columns]
    y_rot = np.matrix([[ math.cos(thetay), 0.0, -math.sin(thetay)],
             [ 0.0, 1.0, 0.0],
             [ math.sin(thetay), 0.0, math.cos(thetay)]]) # y_rot[rows][columns]
    z_rot = np.matrix([[ math.cos(0.0), -math.sin(0.0), 0.0],
             [ math.sin(0.0), math.cos(0.0), 0.0],
             [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

    # Move to grabbing waypoint
    msg = ic.safe_ur_move(c,Pose=dict(iw.grabbing_joints_waypoint),Speed=1.0,CMD=2)

    # Create rotation matrix for current position
    R=z_rot*y_rot*x_rot

    if thetaz!=0:
        # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
        x_rot = np.matrix([[ 1.0, 0.0, 0.0],
                 [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
                 [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]
        z_rot = np.matrix([[ math.cos(thetaz), -math.sin(thetaz), 0.0],
                 [ math.sin(thetaz), math.cos(thetaz), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

        # Cartesian rotation matrix of desired orientation
        R=z_rot*x_rot*R

        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        print rx, ry, rz

        # Rotate around tool centre point defined by tcp_2
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)

        # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
        z_rot = np.matrix([[ math.cos(orientation), -math.sin(orientation), 0.0],
                 [ math.sin(orientation), math.cos(orientation), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

        # Cartesian rotation matrix of desired orientation
        R=z_rot*R

        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        print rx, ry, rz

        # Rotate around tool centre point defined by tcp_2
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)
    else:
        # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
        x_rot = np.matrix([[ 1.0, 0.0, 0.0],
                 [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
                 [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]
        z_rot = np.matrix([[ math.cos(orientation), -math.sin(orientation), 0.0],
                 [ math.sin(orientation), math.cos(orientation), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

        # Cartesian rotation matrix of desired orientation
        R=z_rot*x_rot*R

        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        print rx, ry, rz

        # Rotate around tool centre point defined by tcp_2
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)
