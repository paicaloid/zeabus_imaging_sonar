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

#from sonar.msg import sonar_msg
#from _sonar_msg import *
#from zeabus_imaging_sonar.sr53v import sonar_srv
#from _sonar_srv import *
from zeabus_imaging_sonar.msg import sonar_msg
from zeabus_imaging_sonar.srv import sonar_srv

# Instantiate CvBridge
bridge = CvBridge()

height, width = 660, 768
#height, width = 330, 384
Img_frame, preImg_gray = None, None
count = 0
index = 0
p0, p1 = [], []
new_pos, old_pos = [], []
good_new, good_old = [], []

r_out, theta_out = [], []
#x_out, y_out = [], []

r_thres = 14

r_met = 0
status = False
theta_met = 0

mask = None

def findobject(Img_gray):
	global imgBlur, kernel, npower, ratio, imgcfar, reEdge, opening, theta_out, r_out
#x_out, y_out
	imgBlur = cv2.medianBlur(Img_gray,5)
	imgBlur = imgBlur.astype('float32')
	kernel = np.ones((11,11),'float32')/102.0
	kernel[4:7,4:7] = 0
	npower = cv2.filter2D(imgBlur,cv2.CV_8UC1,kernel)
	npower = npower.astype('float32')+1.0
	ratio = imgBlur/npower
	imgcfar = (ratio>1.45)*255
	imgcfar = imgcfar.astype('uint8')

	#remove noise on edge 
	deEdge = np.zeros((height,width), np.uint8)
	deEdge = cv2.rectangle(deEdge,(0,0),(height,width),(255,255,255),400)
	reEdge = imgcfar-deEdge
	
	#opening
	kernelOpen = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(reEdge, 2, kernelOpen)
	
#	x_out = []
#	y_out = []
	r_out = []
	theta_out = []
	#find center pixel  
	ret,thresh = cv2.threshold(opening,127,255,0)
	contours = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for contour in contours[1] :
		x = contour.mean(0)[0,0]
		y = contour.mean(0)[0,1]
		print 'x = ''%.2f'%x,'m , y = ''%.2f'%y, 'm'
		
		
		#find location
		theta = (2/width)*60*(x-(width/2))
		r = (height-y)
		#print 'r = ''%.2f'%r,', theta = ''%.2f'%theta
		xnew = r*math.sin(math.radians(theta))+(1270/2)
		ynew = 680-r*math.cos(math.radians(theta))
		print 'xnew = ''%.2f'%xnew,'m , ynew = ''%.2f'%ynew, 'm'
		print '----------------------------------------'
		
		#find answer r,theta
		X = xnew
		Y = ynew
		delX = X-(1270/2)
		delY = 680-Y
		r_new = ((math.sqrt(math.pow(delX,2)+math.pow(delY,2)))*18)/620
		theta_new = math.degrees(math.atan((math.fabs(delX)/delY)))
		#x_out.append(xnew)
		#y_out.append(ynew)
		r_out.append(int(r_new))
		# theta_out.append(theta_new)
		if delX >= 0 :
			theta_out.append(theta_new)
			print 'r = ''%.2f'%r_new ,'m , theta = ''%.2f'%theta_new,'degrees'
			print '########################################'
			

		else  :
			theta_out.append(-1*theta_new)
			print 'r = ''%.2f'%r_new ,'m , theta = - ''%.2f'%theta_new,'degrees'
			print '########################################'
			
	pubIm(opening)

	
	

	
	return r_new,theta_new

def pubIm(im):
	img_show = np.array(im, np.uint8)
	msg1 = CompressedImage()
	msg1.format = "jpeg"
	msg1.header.stamp = rospy.Time.now()
	msg1.data = np.array(cv2.imencode('.jpg', img_show)[1]).tostring()
	pub.publish(msg1)

'''def meters(x,y,x_ref, y_ref):
	global r_met, theta_met, status
	pix_met = float(20/y_ref)
	print pix_met
	y_met, x_met = y*pix_met, x*pix_met
	x_ref, y_ref = x_ref*pix_met, y_ref*pix_met
	r = np.sqrt(np.power(x_met - x_ref, 2) + np.power(y_met - y_ref, 2))
	theta = ((math.atan2((x_ref - x_met), (y_ref - y_met))) * 360) / (2 * np.pi)
	theta = theta
	return r, theta'''

def Process():
	global Img_frame, preImg_gray, Img_show, count, p0, p1, good_new, good_old, r_met, theta_met, status, mask, new_pos, old_pos
	global r_out, theta_out
	new_pos, old_pos = [], []
	last_frame = False
	res = sonar_msg()  

	while(Img_frame is None):
		print "Img_frame: None"
		rospy.sleep(0.01)
		continue
	
	Img_gray = cv2.cvtColor(Img_frame, cv2.COLOR_BGR2GRAY)
	p0 = findobject(Img_gray)
	#r_out,theta_out = Tracking(p0)
	res.r = r_out
	res.theta = theta_out
	res.status = status
	print r_out
	print theta_out
	print status
	cv2.waitKey(1)
	return res
	
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
	print("finished setup")
	rospy.spin()
