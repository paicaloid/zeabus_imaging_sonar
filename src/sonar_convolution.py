#!/usr/bin/env python
from __future__ import division
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

from sonar.msg import sonar_msg
from sonar.srv import sonar_srv

bridge = CvBridge()

Img_frame = None

index = 0

mask = None

def Convolution():
	global Img_frame
	print("++++")

	filter_size = 19
	#wn = np.zeros((filter_size,1), dtype = np.int8)
	wn = np.zeros((1,filter_size))
	#print wn
	for i in range (0,7):
		wn[0][i] = 1
		wn[0][i+(19-7)] = 1
	wn = wn/19
	#print wn

	kernel = np.ones((7,7))
	#print kernel
	for i in range (0,3):
		for j in range (0,3):
			kernel[i+2][j+2] = 0
			

	print kernel

	Img_frame = cv2.filter2D(Img_frame,-1,kernel/49)
	rth, Img_frame = cv2.threshold(Img_frame, 70, 255, 0)
	cv2.imshow('SAM', Img_frame)
	cv2.waitKey(30)
	

	return


def image_callback(ros_data):
	global Img_frame, index
	index += 1
	#print "index = %s" %index
	try:
		#Img_frame = cv2.resize(bridge.imgmsg_to_cv2(ros_data, "bgr8"),(width, height))
		Img_frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
		Img_frame = cv2.cvtColor(Img_frame, cv2.COLOR_RGB2GRAY)
		rth, Img_frame = cv2.threshold(Img_frame, 70, 255, 0)
		Convolution()
	except CvBridgeError, e:
		print (e)

def tracking_callback(msg):
	#print msg
	#print "tracking callback"
	#return Process()
	return Convolution()

if __name__ == '__main__':
	rospy.init_node('SonarTracking', anonymous=True)
	
	pub = rospy.Publisher('/image/Tracking', CompressedImage, queue_size=1)
    
	subTopic = '/imaging_sonar'
	sub = rospy.Subscriber(subTopic, Image, image_callback,  queue_size = 1)
	#rospy.Service('/sonar_image', sonar_srv(), tracking_callback)
	rospy.spin()		