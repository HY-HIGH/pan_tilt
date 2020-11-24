#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2, sys, time, os
import rospy
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

# Set up the CascadeClassifier for face tracking
#cascPath = 'haarcascade_frontalface_default.xml' # sys.argv[1]
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
faceCascade = cv2.CascadeClassifier(cascPath)
####################################수정 부분#################################################
center_position=MsgState() 
bridge=CvBridge()
pub = rospy.Publisher('rasimage', Image, queue_size=10)
def centerpositonCB(msgdata):
    """To updateDrone's state 
    """
    global center_position
    center_position=msgdata
class State():
    X_MID   = 0.0
    Y_MID   = 0.0
    BOX_SIZE= 0.0   
class sub_center_position:
    def __init__(self):
        self.state_sub = rospy.Subscriber('data_state',MsgState,callback=centerpositonCB)
    def getposition(self):
        _centerpos = State()
        _centerpos.X_MID=center_position.x_mid 
        _centerpos.Y_MID=center_position.y_mid

        print ("\n-----Parameters-----")
        print ("X_MID   :", _centerpos.X_MID)
        print ("Y_MID   :", _centerpos.Y_MID)



# Set up the capture with our frame size
video_capture = cv2.VideoCapture(0)

video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
time.sleep(2)

# Turn the camera to the default position
pan(cam_pan-90)
tilt(cam_tilt-90)
light_mode(WS2812)


if __name__ == "__main__":
    center_pose=sub_center_position()
    rospy.init_node('sub_center_position', anonymous=False)
    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()
        # This line lets you mount the camera the "right" way up, with neopixels above
        frame = cv2.flip(frame, -1)
        frame = cv2.flip(frame, 1)
        rasimage = frame
        rasimage_msg = bridge.cv2_to_imgmsg(rasimage, encoding="passthrough")
        rasimage_msg.encoding="rgb8"
        pub.publish(rasimage_msg)
        
        #위치 포지셔닝 
        x = (1-center_position.x_mid) * FRAME_W
        y = center_position.y_mid * FRAME_H 

        # Correct relative to center of image
        turn_x  = float(x - (FRAME_W/2))
        turn_y  = float(y - (FRAME_H/2))

        # Convert to percentage offset
        turn_x  /= float(FRAME_W/2)
        turn_y  /= float(FRAME_H/2)

        # Scale offset to degrees
        turn_x   *= 2.5 # VFOV
        turn_y   *= 2.5 # HFOV
        cam_pan  += -turn_x
        cam_tilt += turn_y

        #print(cam_pan-90, cam_tilt-90)
        print(x,y)
        print(turn_x,turn_y)
        # Clamp Pan/Tilt to 0 to 180 degrees
        cam_pan = max(0,min(180,cam_pan))
        cam_tilt = max(0,min(180,cam_tilt))

        # Update the servos
        pan(int(cam_pan-90))
        tilt(int(cam_tilt-90))

        
################################################
################################################

        # frame = cv2.resize(frame, (540,300))
        # frame = cv2.flip(frame, 1)
    
        # Display the image, with rectangle
        # on the Pi desktop 
        # cv2.imshow('Video', frame)
        center_pose.getposition()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()
