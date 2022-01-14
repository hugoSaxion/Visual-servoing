#!/usr/bin/env python

"""
PROJECT ROBOTICS - SAXION
2021

ROS script to determine distance between drone and object.
This will be done by determining the spatial resolution of
the feature. This can be done because you know how many pixels
the feature is on the screen and the deminsions in real life.

The determenined output value is the mean of the last 5 distances
measured in order to prevent outliers. The first 5 distances
are 100 [cm] in stead of 0 to prevent dangerous movements of the 
drone.

Input (ch_scan):
- frame width (im_w) [pixels]
- frame height (im_h) [pixels]
- Field of view (FoV) [deg]
- heigth of feature (h_p) [pixels]
- width of feature (w_p) [pixels]

Output (ch_distance):
- distance from camera to feature (z) [cm]
"""

import rospy
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import String
import math

# define variables
# variables that will be given with input from other nodes
FoV = 0                 #Field of View camera
im_w = 0; im_h = 0      #heigth and width of frame [pixels]
h_p = 0; w_p = 0        #heigth and width of object in pixels
#input
h_r = 15; w_r = 15        #heigth and width of actual object [cm]

# list that will help determine the mean distance over 5 frames
# start values are 100 [cm], to prevent a value that is too close
# to the object
z_list = [100,100,100,100,100]


def mean_list(last_z, new_z):
    """
    Determine distance by taking the mean of the last
    5 inputs. This prevents outliers.
    last_z: must be a list of x lenght
    new_z: must be a int 
    """
    last_z.pop(0)               #remove first item list
    last_z.append(new_z)        #add last measurement
    avg_z= sum(last_z) / len(last_z)    #calculete average value of list
    return avg_z


def callback_location(data):
    """
    Get the input data from the node reading the camera
    """
    global FoV, im_w, im_h, h_p, w_p

    #make list for all data of input
    input = []
    #split the data sperarated by _ and set in list
    for i in data.data.split('_'):
        input.append(i)

    # get data out of input list
    im_w = float(input[0])
    im_h = float(input[1])
    FoV = float(input[2])
    h_p = float(input[5])
    w_p = float(input[6])


def det_distance(rate):
    """
    Determine the changed distance
    """
    while not rospy.is_shutdown():
        #determine distance change
        if h_p != 0:
            #calculate distance
            SR = (w_r)/h_p           #spatial resolution [cm/p]
            c_y = SR*im_h/2
            phi = math.atan(im_h/im_w)  #phi
            d_d = c_y/math.sin(phi)
            z_new = d_d/math.tan(FoV*math.pi/180/2)

            # z = mean_list(z_list,z_new)
            z = z_new*1.2

            #output
            pub = rospy.Publisher('ch_distance', String, queue_size=10)
            rospy.loginfo(str(int(z)))
            pub.publish(str(int(z)))        #publish in [cm]
        rate.sleep()


def startup():
    #name node
    rospy.init_node('distance_scanning', anonymous=True)
    #recieve "ch_scan"
    rospy.Subscriber("ch_scan", String, callback_location)
    #rate
    rate = rospy.Rate(30) # 10hz
    #main function
    det_distance(rate)


if __name__ == '__main__':
    startup()