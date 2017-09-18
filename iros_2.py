#!/usr/bin/env python
# Scripts for iros challenge 2: lay out silverware
#                               re-stow silverware
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
    print "2"