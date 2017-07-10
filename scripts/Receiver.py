#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class receive_img:
	def __init__(self):
		self.image = None
		self.state = 0
		self.subTopic = 'sam'
		self.sub = rospy.Subscriber(self.subTopic, CompressedImage, self.callback,  queue_size=1)

	def callback(self, ros_data):
		print "Receive Image!!"
		np_arr = np.fromstring(ros_data.data, np.uint8)
		self.image = cv2.imdecode(np_arr, 1)	

	def show(self):
        	while(self.image is None):
			rospy.sleep(0.01)
		
		print "I get it!!"
		cv2.imshow('SAM', self.image)
		cv2.waitKey(0)

if __name__ == '__main__':
	rospy.init_node('receiver', anonymous=True)
	##__Create Object__
	try:
		img = receive_img()
		img.show()
	except KeyboardInterrupt:
		print("Shutting down")
