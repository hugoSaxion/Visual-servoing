#!/usr/bin/env python

"""
PROJECT ROBOTICS - SAXION
2021

ROS script to recieve information sent by "ch_scan". "ch_scan" contains
the coordinates (x and y) of a feature per frame. The distance
parameter send by "ch_distance"  will also be recieved.

All the data will be used to determine the real movement of the object in
comparison with the camera. The distance, send by "ch_distance" will be used to
determine the distance.

Input (ch_distance):
- distance from camera to feature (z) [cm]

Input (ch_scan):
- frame width (im_w) [pixels]
- frame height (im_h) [pixels]
- Field of view (FoV) [deg]
- change of horizontal location from first detection point (x_c) [pixels]
- change of vertical location from first detection point (y_c) [pixels]

Output (ch_mov):
- x movement (x) [cm]
- y movement (y) [cm]
- z movement (z) [cm]
"""

import rospy
from std_msgs.msg import String
import numpy as np
import math

# define variables
# variables that will be given with input from other nodes
z = 0; im_w = 0; im_h = 0; FoV = 0
x_new = 0; y_new = 0
spat_res = 0

# list that will help determine the mean coordinates over 5 frames
x_list = [0,0,0,0,0]
y_list = [0,0,0,0,0]

def callback_distance(data):
    "Get the data from the node reading the distance sensor"
    global z
    z = float(data.data)/100      #convert distance from cm to m


def callback_location(data):
    """
    Get the data from the node reading the camera.
    Here the image width, heigth and Field of View will be given
    Also the new x and y coordinates will be given in pixels
    """
    global im_w, im_h, FoV, x_new, y_new

    input = []
    #split the data sperarated by _ and set in list
    for i in data.data.split('_'):
        input.append(i)

    im_w = float(input[0])
    im_h = float(input[1])
    FoV = float(input[2])
    x_new = float(input[3])
    y_new = float(input[4])


def mean_loc(lastx_co, new_co):
    """
    Determine location by taking the mean of the last
    5 location coordinets. This prevents outliers.
    lastx_co: must be a list of x lenght
    new_co: must be a int 
    """
    lastx_co.pop(0)               #remove first item list
    lastx_co.append(new_co)       #add last value to list
    avg_co = sum(lastx_co) / len(lastx_co)      #calculete average value list
    return avg_co


def det_location(rate):
    while not rospy.is_shutdown():
        #determine location change from [pixels] to [m]
        if FoV != 0:
            # determine spatial resolution
            d_d = z*math.tan(FoV*math.pi/180/2)
            phi = math.atan(im_h/im_w)      #phi
            d_x = d_d*math.cos(phi)         #1/2 width in [m]
            SR = 2*d_x/im_w                 #spatial resolution (mm/pixel)

            x = mean_loc(x_list,x_new*SR)
            y = mean_loc(y_list,y_new*SR)

            #output
            print(str(int(z*100)))
            pub = rospy.Publisher('ch_mov', String, queue_size=10)      #CAN BE MOVED TO STARTUP()
            # rospy.loginfo(str(round(x*100,1)) + '_' + str(round(y*100,1)) + '_' + str(z*100))
            pub.publish(str(round(x*100,1)) + '_' + str(round(y*100,1)) + '_' + str(z*100))       #publish in [cm]
        rate.sleep()


def startup():
    #name node
    rospy.init_node('det_movement', anonymous=True)
    #recieve "ch_distance"
    rospy.Subscriber("ch_distance", String, callback_distance)
    #recieve "ch_scan"
    rospy.Subscriber("ch_scan", String, callback_location)
    #rate
    rate = rospy.Rate(30) # 10hz
    #main function
    det_location(rate)

if __name__ == '__main__':
    startup()