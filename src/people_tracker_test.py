#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2, sys, time, os
import rospy
from datetime import datetime
from pantilthat import *
from pan_tilt.msg import MsgState
from sensor_msgs.msg import Image #이미지 캡쳐
from cv_bridge import CvBridge, CvBridgeError

# Load the BCM V4l2 driver for /dev/video0
os.system('sudo modprobe bcm2835-v4l2')
# Set the framerate ( not sure this does anything! )
os.system('v4l2-ctl -p 8')

# Frame Size. Smaller is faster, but less accurate.
# Wide and short is better, since moving your head
# vertically is kinda hard!
FRAME_W = 180
FRAME_H = 100

# Default Pan/Tilt for the camera in degrees.
# Camera range is from -90 to 90 # 기본 angle

cam_pan = 90
cam_tilt = 60

####################################수정 부분#################################################

# cascPath = sys.argv[2]
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
 
# Create the haar cascade
faceCascade = cv2.CascadeClassifier(cascPath)
 
start_time = datetime.now()
 
# 계산 반복 횟수 (한번만 처리하려면 아래를 1로 하거나 for문을 제거하세요)
iteration_count = 100

video_capture = cv2.VideoCapture(0)

video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

for cnt in range(0, iteration_count):
 
    # Read the image
    # image = cv2.imread(imagePath)
    gray = cv2.cvtColor(video_capture, cv2.COLOR_BGR2GRAY)
 
    # Detect faces in the image
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,     # 이미지에서 얼굴 크기가 서로 다른 것을 보상해주는 값
        minNeighbors=5,    # 얼굴 사이의 최소 간격(픽셀)입니다
        minSize=(30, 30),   # 얼굴의 최소 크기입니다
    )
 
    # 검출된 얼굴 주변에 사각형 그리기
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
 
end_time = datetime.now()
elapsed_time = end_time - start_time
average_time = elapsed_time / iteration_count
 
print "Elapsed Time: %s sec" % elapsed_time
print "Average Time: %s sec" % average_time
 
# 얼굴을 검출한 이미지를 화면에 띄웁니다
cv2.imshow("Face Detected", image)
 
# 아무 키나 누르면 빠져나옵니다
cv2.waitKey(0)