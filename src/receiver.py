#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
class receive_img:
	def __init__(self):
		self.image = None
		self.state = 0
		self.subTopic = '/imaging_sonar'
		self.sub = rospy.Subscriber(self.subTopic, Image, self.callback,  queue_size=1)

	def callback(self, ros_data):
		print "Receive Image!!"
		try:
			Img_frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
		except CvBridgeError, e:
			print (e)

	def show(self):
		while(self.image is None):
			rospy.sleep(0.01)
		
		print "I get it!!"
		cv2.imshow('SAM', self.image)
		cv2.waitKey(0)

if __name__ == '__main__':
	rospy.init_node('SonarTracking', anonymous=True)
	##__Create Object__
	try:
		img = receive_img()
		img.show()
		print "+++++"
	except KeyboardInterrupt:
		print("Shutting down")
