#!/usr/bin/env python
from __future__ import division
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from zeabus_imaging_sonar.msg import sonar_msg
from zeabus_imaging_sonar.srv import sonar_srv

# Instantiate CvBridge
bridge = CvBridge()

width, height = 500, 200
Img_frame, preImg_gray = None, None
count = 0
index = 0
p0, p1 = [], []
new_pos, old_pos = [], []
good_new, good_old = [], []

r_out, theta_out = [], []

r_thres = 10

r_met = 0
status = False
theta_met = 0

mask = None

def CornerDetect(Img_gray):
	global Img_frame, p0, mask
	feature_params = dict( maxCorners   = 50,
                           qualityLevel = 0.3,
                           minDistance  = 10,
                           blockSize    = 7 )
	#blur = cv2.medianBlur(Img_gray, 5)
	rth, Img_gray_th = cv2.threshold(Img_gray, 70, 255, 0)
	p0 = cv2.goodFeaturesToTrack(Img_gray_th, mask=None, **feature_params)
	"""for i in range (0,len(p0)):
		cv2.circle(Img_frame, (p0[i][0][0], p0[i][0][1]), 5, (142, 164, 255), -1)
	cv2.imshow('Corner', Img_frame)
	Img_frame = cv2.cvtColor(Img_gray, cv2.COLOR_GRAY2BGR)"""
	mask = np.zeros_like(Img_frame)
	return p0, mask

def OpticalFlow(Img_gray):
	global preImg_gray, p0
	lk_params = dict( winSize  = (30,30),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS |cv2.TERM_CRITERIA_COUNT, 10, 0.03))
	p1, st, err = cv2.calcOpticalFlowPyrLK(preImg_gray, Img_gray, p0, None, **lk_params)
	return p1, st

def FindObject(Img_gray):
	global count, r_met, r_thres, theta_met, status, mask, new_pos, old_pos
	global r_out, theta_out
	r_met = 0 
	count_newLast, index = 0, 0
	est_obj = []
	find_obj = False
	status = False
	row, col = Img_gray.shape[0], Img_gray.shape[1]
	x_ref, y_ref = col/2, row

	r_out, theta_out = [], []

	Img_sh = cv2.cvtColor(Img_gray, cv2.COLOR_GRAY2BGR)
	if len(new_pos) == 0:
		new_pos = old_pos
	for i in range (0,len(new_pos)):
		count_newLast += 1
		x, y = new_pos[i][0], new_pos[i][1]
		theta = ((math.atan2((x_ref - x), (y_ref - y))) * 360) / (2 * np.pi)
		r = np.sqrt(np.power(x - x_ref, 2) + np.power(y_ref - y, 2))
		r_met, theta_met = meters(x, y, x_ref, y_ref)
		#print theta_met , r_met
		if(-45 < theta < 45):  ##Limit theta
			est_obj.append((x, y))
			find_obj = True
	if find_obj:	
		for j in range (0,len(est_obj)):
			Obj_x, Obj_y = est_obj[j][0], est_obj[j][1]
			if ((j > 0) and (pre_est_x == Obj_x) and (pre_est_y == Obj_y)): ##Avoid Duplicate Position
				print "Duplicate"
			else:
				r_met, theta_met = meters(Obj_x, Obj_y, x_ref, y_ref)
				theta_met = theta_met + 90
				r_met, theta_met = int(round(r_met)), int(round(theta_met))
				if r_met <= r_thres:
					print "Estimate # %d frame Object # %d: R = %.2f meters, Theta = %.2f" % (count, j + 1, r_met, theta_met)
					r_out.append(r_met)
					theta_out.append(theta_met)
					cv2.circle(Img_sh, (Obj_x, Obj_y), 5, (102, 255, 204), -1)
				pre_est_x, pre_est_y = Obj_x, Obj_y
				pubIm(Img_sh)
		status = True
		#cv2.imshow('Object n frame', Img_sh)
	else:
		print "Can't find Object"
	return r_out, theta_out

def pubIm(im):
	img_show = np.array(im, np.uint8)
	msg1 = CompressedImage()
	msg1.format = "jpeg"
	msg1.header.stamp = rospy.Time.now()
	msg1.data = np.array(cv2.imencode('.jpg', img_show)[1]).tostring()
	pub.publish(msg1)

def meters(x,y,x_ref, y_ref):
	global r_met, theta_met, status
	pix_met = float(20/y_ref)
	print pix_met
	y_met, x_met = y*pix_met, x*pix_met
	x_ref, y_ref = x_ref*pix_met, y_ref*pix_met
	r = np.sqrt(np.power(x_met - x_ref, 2) + np.power(y_met - y_ref, 2))
	theta = ((math.atan2((x_ref - x_met), (y_ref - y_met))) * 360) / (2 * np.pi)
	theta = theta
	return r, theta

def Process():
	global Img_frame, preImg_gray, count, p0, p1, good_new, good_old, r_met, theta_met, status, mask, new_pos, old_pos
	global r_out, theta_out
	new_pos, old_pos = [], []
	last_frame = False
	res = sonar_msg()  #!!

	while(Img_frame is None):
		print "Img_frame: None"
		rospy.sleep(0.01)
		continue

	count += 1
	left = int(round((Img_frame.shape[1]/2)-(Img_frame.shape[1]/4)))
	right = int(round((Img_frame.shape[1]/2)+(Img_frame.shape[1]/4)))
	
	Img_gray = cv2.cvtColor(Img_frame, cv2.COLOR_BGR2GRAY)
	 
	if count == 1:
		print "Corner", count
		p0, mask = CornerDetect(Img_gray)
		preImg_gray = Img_gray.copy()
	else:
		print "optical", count
		ref_gray = Img_gray.copy()
		p1, st = OpticalFlow(Img_gray)
		good_new = p1[st == 1]
		good_old = p0[st == 1]
		for i, (new, old) in enumerate(zip(good_new, good_old)):
			a, b = new.ravel()
			c, d = old.ravel()
			if (a > left) and (a < right):
				new_pos.append((a,b))
				old_pos.append((c,d))
				cv2.line(mask, (a, b), (c, d), (255, 212, 128), 2)
				cv2.circle(Img_frame, (a, b), 5, (77, 77, 255), -1)
		Img_re = cv2.add(Img_frame, mask)
		p0 = good_new.reshape(-1, 1, 2)
		preImg_gray = ref_gray.copy()
	r_out,theta_out = FindObject(Img_gray)
	res.r = r_out
	res.theta = theta_out
	res.status = status
	cv2.waitKey(1)
	return res
	
def Convolution(Img_gray):

	kernel = np.ones((7,7))
	#print kernel
	for i in range (0,3):
		for j in range (0,3):
			kernel[i+2][j+2] = 0

	# print kernel
	rth, Img_conv = cv2.threshold(Img_gray, 70, 255, 0)
	Img_conv = cv2.filter2D(Img_conv,-1,kernel/49)
	rth, Img_conv = cv2.threshold(Img_conv, 70, 255, 0)
	#cv2.imshow('SAM', Img_frame)
	#cv2.waitKey(30)

	return Img_conv

def image_callback(ros_data):
	global Img_frame, width, height, index
	index += 1
	#print "index = %s" %index
	try:
		Img_frame = cv2.resize(bridge.imgmsg_to_cv2(ros_data, "bgr8"),(width, height))
	except CvBridgeError, e:
		print (e)

def tracking_callback(msg):
	print msg
	print "tracking callback"
	return Process()

if __name__ == '__main__':
	rospy.init_node('SonarTracking', anonymous=True)
	## Publish for show an image
	pub = rospy.Publisher('/image/Tracking', CompressedImage, queue_size=1)
	subTopic = "/imaging_sonar"
	sub = rospy.Subscriber(subTopic, Image, image_callback,  queue_size = 1)
	rospy.Service('/sonar_image', sonar_srv(), tracking_callback)
	rospy.spin()
