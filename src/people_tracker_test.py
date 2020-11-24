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


cam_pan = 0
cam_tilt = 50
pan(cam_pan) # Turn the camera to the default position
tilt(cam_tilt)
light_mode(WS2812)

if __name__ == "__main__":
    rospy.init_node('pantilt_node', anonymous=False)

    while not rospy.is_shutdown():
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.flip(gray, -1)
        gray = cv2.flip(gray, 1)
        faces = face_cascade.detectMultiScale(gray, 1, 5)



        for (x,y,w,h) in faces:
            gray = cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            
        rasimage = gray
        rasimage_msg = bridge.cv2_to_imgmsg(rasimage, encoding="passthrough")
        rasimage_msg.encoding="mono8"
        rasimage_pub.publish(rasimage_msg)

        # cv2.imshow('img',img)
        # k = cv2.waitKey(30) & 0xff
        # if k == 27:
        #     break

    cap.release()
    # cv2.destroyAllWindows()