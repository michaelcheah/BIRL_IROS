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
    print "10"