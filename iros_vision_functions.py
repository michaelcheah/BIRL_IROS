import iros_vision_tools as ivt
import numpy as np
import copy
import os
import cv2
from scipy.spatial.distance import cdist
from matplotlib import pyplot as plt


def cup_saucer(test_image, show=False):
    test_img = copy.copy(test_image)
        
    CAL_PARAM = {'thresh': [25, 60],
            'radius': [30,85]}
    circles, cimg = ivt.find_circles(copy.copy(test_img), 6, param=CAL_PARAM, blur=1, show=False)

    CAL_PARAM = {'thresh': [105, 140],
                'radius': [30,85]}
    circles2, cimg = ivt.find_circles(copy.copy(test_img), 2, param=CAL_PARAM, blur=1, show=False)

    print len(circles[0])
    print len(circles2[0])

    table_circles = {}
    store = []
    for j,i in enumerate(circles2[0]):
        circle_info = {}
        circle_info["id"] = j
        coords = np.array([i[:-1]])
        radius = i[-1]
        circle_info["circle"]=i
        num = 0
        for k in (circles[0]):
            #print np.array([k[:-1]])
            if cdist(np.array([k[:-1]]),coords)<radius:
                num = num+1
        circle_info["num_circles"]=num
        store.append(circle_info)
    
    for member in store:
        if member["num_circles"]>3:
            color = (0,255,0)
            table_circles["saucer"]=member
        else:
            color = (0,0,255)
            table_circles["mug"]=member
        print member['circle']

        cv2.circle(cimg,(int(member["circle"][0]),int(member["circle"][1])),int(member["circle"][2]),color,1)
                    # draw the center of the ci~rcle
        cv2.circle(cimg,(int(member["circle"][0]),int(member["circle"][1])),2,color,1)
    if show==True:
        plt.imshow(cimg)
        plt.show()

    return table_circles

def find_spoon(image, show=True):
    img = copy.copy(image)
    edged, edg_img, cnts, hierarchy=ivt.extract_contours(copy.copy(img), 
                                                        min_thresh=25, 
                                                        max_thresh=240, 
                                                        blur = 5, dilate=3, erode=0, 
                                                        cnt_mode = cv2.RETR_TREE)
    CAL_PARAM = {'thresh': [75, 100],
                 'radius': [30,45]}
    minsize=500
    mindistance = 1000
    box_minsize = 1200
    
    circles, cimg = ivt.find_circles(copy.copy(img), 1, param=CAL_PARAM, blur=1, show=False)

    show_img = copy.copy(img)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_RGB2GRAY)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_GRAY2RGB)
    
    for cnt in cnts:
        current_outer_contour = []
        for points in cnt:
            current_outer_contour.append(points[0])

        distance = cdist(np.array([[circles[0][0][0],circles[0][0][1]]]),current_outer_contour)

        if cv2.contourArea(cnt) < minsize:
            print("Object at #{} REJECTED because CONTOUR not big enough: ".format(cnt[0]), cv2.contourArea(cnt))
            continue

        if distance[0][0] > mindistance:
            print("Object at #{} REJECTED because not CLOSE ENOUGH: ".format(cnt[0]), distance[0][0])
            continue
        mindistance = distance[0][0]

        box = ivt.extract_minBox(cnt)
        box_area = abs((box[0][0]-box[2][0])*(box[1][1]-box[0][1]))
        
        if box_area < box_minsize:
            print("Object at #{} REJECTED because BOX not big enough: ".format(cnt[0]), box_area)
            continue
        fnode = ivt.farthest_node([circles[0][0][0],circles[0][0][1]], current_outer_contour)

    print "FNODE: ", fnode
    print "CIRCLE:", circles
    cv2.circle(show_img,(int(circles[0][0][0]),int(circles[0][0][1])),3,(0,255,0),5)
    cv2.circle(show_img, (fnode[0][0], fnode[0][1]), 3, (0,255,255),5)
    
    if show:
        plt.imshow(show_img)
        plt.figure()
        plt.imshow(cimg)
        plt.show()
    return circles[0], fnode 