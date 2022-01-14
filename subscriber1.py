#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data.data)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('ch_mov', String, callback)

    rate = rospy.Rate(10) #10hz
    rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

# https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29