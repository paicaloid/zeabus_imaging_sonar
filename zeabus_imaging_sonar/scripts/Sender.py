#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class send_img:
	def __init__(self):
		self.image = cv2.imread("/home/johny/Desktop/SAMUEL.jpg", 0)
		self.state = 0
		self.pubTopic = 'sam'
		self.pub = rospy.Publisher(self.pubTopic, CompressedImage, queue_size=1)
	
	def sending(self):
		print "sending"
		while(self.image is None):
			rospy.sleep(0.01)

		self.image = cv2.imread("/home/johny/Desktop/SAMUEL.jpg")

		msg = CompressedImage()
		msg.format = "jpeg"

		while not rospy.is_shutdown():
				self.state += 1

				print(self.state)
				if self.state % 3 == 0:
					print "Sending Image!!"
					msg.header.stamp = rospy.Time.now()
					msg.data = np.array(cv2.imencode('.jpg', self.image)[1]).tostring()
					self.pub.publish(msg)
					self.state = 0

if __name__ == '__main__':
	rospy.init_node('sender', anonymous=True)
	##__Create Object__
	img = send_img()
	img.sending()

