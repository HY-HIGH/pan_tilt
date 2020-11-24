#!/usr/bin/env python
#-*- coding:utf-8 -*-


import numpy as np
import cv2, sys, time, os
import rospy
from pantilthat import *
from pan_tilt.msg import MsgState
from sensor_msgs.msg import Image #이미지 캡쳐
from cv_bridge import CvBridge, CvBridgeError

face_cascade = cv2.CascadeClassifier('./pretrained_data/data/haarcascades/haarcascade_frontalface_default.xml')
cap = cv2.VideoCapture(0)


# Load the BCM V4l2 driver for /dev/video0
os.system('sudo modprobe bcm2835-v4l2')
# Set the framerate ( not sure this does anything! )
os.system('v4l2-ctl -p 8')


bridge=CvBridge()
rasimage_pub = rospy.Publisher('rasimage', Image, queue_size=10)


cam_pan = 90
cam_tilt = 60
pan(cam_pan-90) # Turn the camera to the default position
tilt(cam_tilt-90)
light_mode(WS2812)

if __name__ == "__main__":
    rospy.init_node('sub_center_position', anonymous=False)

    while not rospy.is_shutdown():
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        img = cv2.flip(img, -1)
        img = cv2.flip(img, 1)

        rasimage = img
        rasimage_msg = bridge.cv2_to_imgmsg(rasimage, encoding="passthrough")
        rasimage_msg.encoding="rgb8"
        rasimage_pub.publish(rasimage_msg)

        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]

        # cv2.imshow('img',img)
        # k = cv2.waitKey(30) & 0xff
        # if k == 27:
        #     break

    cap.release()
    # cv2.destroyAllWindows()