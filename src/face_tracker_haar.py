# import necessary packages
from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID
import pantilthat as pth
import argparse
import signal
import time
import sys
import cv2
# define the range for the motors
servoRange = (-90, 90)

class PID:
    def __init__(self, kP=1, kI=0, kD=0):
    # initialize gains
    self.kP = kP
    self.kI = kI
    self.kD = kD

	def initialize(self):
    		# initialize the current and previous time
		self.currTime = time.time()
		self.prevTime = self.currTime
		# initialize the previous error
		self.prevError = 0
		# initialize the term result variables
		self.cP = 0
		self.cI = 0
		self.cD = 0

    def update(self, error, sleep=0.2):
    		# pause for a bit
		time.sleep(sleep)
		# grab the current time and calculate delta time
		self.currTime = time.time()
		deltaTime = self.currTime - self.prevTime
		# delta error
		deltaError = error - self.prevError
		# proportional term
		self.cP = error
		# integral term
		self.cI += error * deltaTime
		# derivative term and prevent divide by zero
		self.cD = (deltaError / deltaTime) if deltaTime > 0 else 0
		# save previous time and error for the next update
		self.prevtime = self.currTime
		self.prevError = error
		# sum the terms and return
		return sum([
			self.kP * self.cP,
			self.kI * self.cI,
			self.kD * self.cD])

class ObjCenter:
    def __init__(self, haarPath):
        # load OpenCV's Haar cascade face detector
        self.detector = cv2.CascadeClassifier(haarPath)
    
    def update(self, frame, frameCenter):
    		# convert the frame to grayscale
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		# detect all faces in the input frame
		rects = self.detector.detectMultiScale(gray, scaleFactor=1.05,
			minNeighbors=9, minSize=(30, 30),
			flags=cv2.CASCADE_SCALE_IMAGE)
		# check to see if a face was found
		if len(rects) > 0:
			# extract the bounding box coordinates of the face and
			# use the coordinates to determine the center of the
			# face
			(x, y, w, h) = rects[0]
			faceX = int(x + (w / 2.0))
			faceY = int(y + (h / 2.0))
			# return the center (x, y)-coordinates of the face
			return ((faceX, faceY), rects[0])
		# otherwise no faces were found, so return the center of the
		# frame
		return (frameCenter, None)

# function to handle keyboard interrupt
def signal_handler(sig, frame):
	# print a status message
	print("[INFO] You pressed `ctrl + c`! Exiting...")
	# disable the servos
	pth.servo_enable(1, False)
	pth.servo_enable(2, False)
	# exit
	sys.exit()

def obj_center(args, objX, objY, centerX, centerY):
    	# signal trap to handle keyboard interrupt
	signal.signal(signal.SIGINT, signal_handler)
	# start the video stream and wait for the camera to warm up
	vs = VideoStream(usePiCamera=True).start()
	time.sleep(2.0)
	# initialize the object center finder
	obj = ObjCenter(args["cascade"])
	# loop indefinitely
	while True:
		# grab the frame from the threaded video stream and flip it
		# vertically (since our camera was upside down)
		frame = vs.read()
		frame = cv2.flip(frame, 0)
		# calculate the center of the frame as this is where we will
		# try to keep the object
		(H, W) = frame.shape[:2]
		centerX.value = W // 2
		centerY.value = H // 2
		# find the object's location
		objectLoc = obj.update(frame, (centerX.value, centerY.value))
		((objX.value, objY.value), rect) = objectLoc
		# extract the bounding box and draw it
		if rect is not None:
			(x, y, w, h) = rect
			cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),
				2)
		# display the frame to the screen
		cv2.imshow("Pan-Tilt Face Tracking", frame)
		cv2.waitKey(1)

def pid_process(output, p, i, d, objCoord, centerCoord):
    	# signal trap to handle keyboard interrupt
	signal.signal(signal.SIGINT, signal_handler)
	# create a PID and initialize it
	p = PID(p.value, i.value, d.value)
	p.initialize()
	# loop indefinitely
	while True:
		# calculate the error
		error = centerCoord.value - objCoord.value
		# update the value
		output.value = p.update(error)