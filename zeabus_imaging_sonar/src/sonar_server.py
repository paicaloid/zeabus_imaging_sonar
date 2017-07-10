#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sonar.msg import sonar_msg
from sonar.srv import sonar_srv
sonar_image = None

def server(start):
    global sonar_image
    # try:      
    response = sonar_image(start)
    print response
    return response.theta, response.r, response.status
    # except  rospy.ServiceException, e:
    #     print "Service call failed: %s"%e

def showimg(r, theta):
    im = np.zeros((200,500))
    for i, (ri, thetai) in enumerate (zip(r, theta)):
        cv2.circle(Im, (ri, thetai), 5, (102, 255, 204), -1)
    cv2.imshow('re', Im)
    cv2.waitKey(10)

if __name__ == "__main__":
    print "Waiting"
    start = True
    count = 0
    rospy.wait_for_service('/sonar_image')
    sonar_image = rospy.ServiceProxy('/sonar_image', sonar_srv)
    print 'service start'
    while not rospy.is_shutdown():
	    count += 1
	    if count % 5 == 0:
		    print count    	
		    # response = sonar_image(start)
		    # server(start)    
		    #print "theta = %s, r = %s, status = %r" %(response.theta, response.r, response.status)
		    response = sonar_image(start)
		    print response
