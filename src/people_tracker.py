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
FRAME_W = 720
FRAME_H = 400

# Default Pan/Tilt for the camera in degrees.
# Camera range is from -90 to 90
cam_pan = 90
cam_tilt = 60

# Set up the CascadeClassifier for face tracking
#cascPath = 'haarcascade_frontalface_default.xml' # sys.argv[1]
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
faceCascade = cv2.CascadeClassifier(cascPath)
#수정 부분
center_position=MsgState()
bridge=CvBridge()
pub = rospy.Publisher('rasimage', Image, queue_size=100)
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
#video_capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,  FRAME_W)
#video_capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, FRAME_H)

video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
time.sleep(2)

# Turn the camera to the default position
pan(cam_pan-90)
tilt(cam_tilt-90)
# light_mode(WS2812)

# def lights(r,g,b,w):
#     for x in range(18):
#         set_pixel_rgbw(x,r if x in [3,4] else 0,g if x in [3,4] else 0,b,w if x in [0,1,6,7] else 0)
#     show()
# lights(0,0,0,50)
if __name__ == "__main__":
    center_pose=sub_center_position()
    rospy.init_node('sub_center_position', anonymous=False)
    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()
        # This line lets you mount the camera the "right" way up, with neopixels above
        frame = cv2.flip(frame, 1)
        rasimage = frame
        rasimage_msg = bridge.cv2_to_imgmsg(rasimage, encoding="passthrough")
        rasimage_msg.encoding="rgb8"
        pub.publish(rasimage_msg)
        
        if ret == False:
            print("Error getting image")
            continue

        # # Convert to greyscale for detection
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.equalizeHist( gray )

        # # Do face detection
        # faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
    
        # Slower method 
        # '''faces = faceCascade.detectMultiScale(
        #     gray,
        #     scaleFactor=1.1,
        #     minNeighbors=4,
        #     minSize=(20, 20),
        #     flags=cv2.cv.CV_HAAR_SCALE_IMAGE | cv2.cv.CV_HAAR_FIND_BIGGEST_OBJECT | cv2.cv.CV_HAAR_DO_ROUGH_SEARCH
        # )'''
        
        # lights(50 if len(faces) == 0 else 0, 50 if len(faces) > 0 else 0,0,50)
################################################
################################################
        # for (x, y, w, h) in faces:
        #     # Draw a green rectangle around the face
        #     cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        #     # Track first face
            
        #     # Get the center of the face
        #     x = x + (w/2)
        #     y = y + (h/2)

        #     # Correct relative to center of image
        #     turn_x  = float(x - (FRAME_W/2))
        #     turn_y  = float(y - (FRAME_H/2))

        #     # Convert to percentage offset
        #     turn_x  /= float(FRAME_W/2)
        #     turn_y  /= float(FRAME_H/2)

        #     # Scale offset to degrees
        #     turn_x   *= 2.5 # VFOV
        #     turn_y   *= 2.5 # HFOV
        #     cam_pan  += -turn_x
        #     cam_tilt += turn_y

        #     #print(cam_pan-90, cam_tilt-90)
        #     print(x,y)

        #     # Clamp Pan/Tilt to 0 to 180 degrees
        #     cam_pan = max(0,min(180,cam_pan))
        #     cam_tilt = max(0,min(180,cam_tilt))

        #     # Update the servos
        #     pan(int(cam_pan-90))
        #     tilt(int(cam_tilt-90))

        #     break
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
