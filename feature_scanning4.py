#!/usr/bin/env python

"""
PROJECT ROBOTICS - SAXION
2021

ROS script to scan a qr code using a webcam
and determine its midpoint on the frame.
These points (x and y) will be sent by the channel "ch_scan"
Tracking of the qr code will start when the centre
of the qr code has been in a certain box.
The size of this box can be adjusted with 'n'.

Input:
- Camera frame (FoV = 60 deg)

Output (ch_scan):
- frame width (im_w) [pixels]
- frame height (im_h) [pixels]
- Field of view (FoV) [deg]
- change of horizontal location from middle point (x_c) [pixels]
- change of vertical location from middle point (y_c) [pixels]
- heigth of qr code (h) [pixels]
- width of qr code (w) [pixels]
"""

import rospy
from std_msgs.msg import String
from pyzbar import pyzbar
import time
import cv2

def start_publisher():
    #start publisher ROS and set rospy
    pub = rospy.Publisher('ch_scan', String, queue_size=10)
    rospy.init_node('feature_scanning', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    return pub, rate


def start_camera():
    # initialize the video stream and allow the camera sensor to warm up
    print("[INFO] starting video stream...")
    # src=0: intern webcam; src=2: extern webcam
    cam = cv2.VideoCapture(0)
    time.sleep(2.0)
    return cam


def calibrate(cam):
    #get the camera data
    ret_cal, frame = cam.read()
    #determine image hight and width
    im_h, im_w, k = frame.shape     #pixels
    #determine Field of View
    FoV = 60    #deg
    return im_h, im_w, FoV


def talker():
    #startup
    pub, rate = start_publisher()
    cam = start_camera()
    im_h, im_w, FoV = calibrate(cam)

    #for first detection
    i = False

    while not rospy.is_shutdown():
        # loop over the frames from the video stream
        ret_cal, frame = cam.read()
        # find the qrcode in the frame
        qrcodes = pyzbar.decode(frame)

        #create rectangle (blue) in wich the centre of the qr code must be before start following
        n = .90     #size of box (1 to 0)
        cv2.rectangle(frame, (int(im_w*(1-n)), int(im_h*(1-n))), (int(im_w*n),int(im_h*n)),(225,0,0), 2)
        
        # draw centre point on middle qrcode (blue)
        cv2.circle(frame, (int(im_w*.5), int(im_h*.5)), radius=10, color=(255,0,0), thickness=3)

        for qrcode in qrcodes:
            # extract the bounding box location of the qrcode and draw
            # the bounding box surrounding the qrcode on the image
            # x and y are the coordinates of the upper left corner of the qrcode
            # w and h are the dimensions of the qr code
            (x, y, w, h) = qrcode.rect

            #when the centre of the qr code is or has been in the box
            if i == True or (im_w*(1-n) < x + w*.5 < im_w*n and im_h*(1-n) < y + w*.5 < im_h*n):
                # draw rectangle on frame at borders qr code
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0,0,255), 2)

                # draw centre point on middle qrcode (blue)
                cv2.circle(frame, (int(im_w*.5), int(im_h*.5)), radius=10, color=(255,0,0), thickness=3)
                # draw currecnt detection point on middle qrcode (red)
                cv2.circle(frame, (int(x+.5*w), int(y+.5*h)), radius=10, color=(0,0,255), thickness=3)


                # the qrcode data is a bytes object so if we want to draw it
                # on our output image we need to convert it to a string first
                qrcodeData = qrcode.data.decode("utf-8")
                qrcodeType = qrcode.type
                # draw the qrcode data and qrcode type on the image
                text = "{} ({})".format(qrcodeData, qrcodeType)
                cv2.putText(frame, text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


                # get the x and y displacement seen from the original position
                x_c = x + w*.5 - im_w*.5
                y_c = y + h*.5 - im_h*.5
                # publish shape frame, FoV and middle coordinates (x, y) feature in string seperated by _
                print("[INFO] Found {} qrcode: {}".format(qrcodeType, qrcodeData))
                rospy.loginfo(str(im_w) +'_'+ str(im_h) +'_'+ str(FoV) +'_'+ str(x_c) +'_'+ str(y_c) +'_'+ str(h) +'_'+ str(w))
                pub.publish(str(im_w) +'_'+ str(im_h) +'_'+ str(FoV) +'_'+ str(x_c) +'_'+ str(y_c) +'_'+ str(h) +'_'+ str(w))
                    
                print("TRUE")
                i = True


        # show the output frame
        cv2.imshow("qrcode Scanner", frame)
        key = cv2.waitKey(1) & 0xFF
        rate.sleep()
    
    # close application
    print("[INFO] cleaning up...")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
