#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
import cv2, sys, time, os
from pantilthat import *
from pan_tilt.msg import MsgState
from sensor_msgs.msg import Image #이미지 캡쳐
from cv_bridge import CvBridge, CvBridgeError


class PID:
    def __init__(self, kP=1, kI=0, kD=0):
        # initialize gains
        self.kP = kP
        self.kI = kI
        self.kD = kD
        

if __name__ == "__main__":
    # rospy.init_node('pantilt_node', anonymous=False)

    face_cascade = cv2.CascadeClassifier('./pretrained_data/data/haarcascades/haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)

    # Load the BCM V4l2 driver for /dev/video0
    os.system('sudo modprobe bcm2835-v4l2')
    # Set the framerate ( not sure this does anything! )
    os.system('v4l2-ctl -p 8')
    # bridge=CvBridge()
    # rasimage_pub = rospy.Publisher('rasimage', Image, queue_size=10)

    cam_pan = 0
    cam_tilt = 50
    pan(cam_pan) # Turn the camera to the default position
    tilt(cam_tilt)
    light_mode(WS2812)

    pid = PID(5, 0.5, 0)

    start_time = time.time()
    error_prev = 0.
    time_prev = 0.


    while cap is not None:
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.flip(gray, -1)
        gray = cv2.flip(gray, 1)
        height, width = gray.shape
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        
        for (x,y,w,h) in faces:
            gray = cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]

            error_x  = (x+w/2.0) - (width/2.0)
            error_y  = (y+h/2.0) - (height/2.0)
            error_x  /= (width/2.0) # VFOV
            error_y  /= (height/2.0) # HFOV

            cam_pan  += error_x * 5
            cam_tilt += error_y * 5

            pan(cam_pan) # Turn the camera to the default position
            tilt(cam_tilt)
            
            print("width : ",width , "height : ", height)
            print("box_x : ",x , "box_y : ", y)
            print("x : ", cam_pan, "y : ", cam_tilt)

        # rasimage = gray
        # rasimage_msg = bridge.cv2_to_imgmsg(rasimage, encoding="passthrough")
        # rasimage_msg.encoding="mono8"
        # rasimage_pub.publish(rasimage_msg)

        # cv2.imshow('img',img)
        # k = cv2.waitKey(30) & 0xff
        # if k == 27:
        #     break

    cap.release()
    # cv2.destroyAllWindows()