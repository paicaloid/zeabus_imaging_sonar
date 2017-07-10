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

<<<<<<< HEAD
=======
def showimg(r, theta):
    im = np.zeros((200,500))
    for i, (ri, thetai) in enumerate (zip(r, theta)):
        cv2.circle(Im, (ri, thetai), 5, (102, 255, 204), -1)
    cv2.imshow('re', Im)
    cv2.waitKey(10)
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

if __name__ == "__main__":
    print "Waiting"
    start = True
    count = 0
    rospy.wait_for_service('/sonar_image')
    sonar_image = rospy.ServiceProxy('/sonar_image', sonar_srv)
    print 'service start'
    while not rospy.is_shutdown():
<<<<<<< HEAD
        count += 1
        time.sleep(1)
        response = sonar_image(start)
        """for i, (ri, thetai) in enumerate(zip(response.r, response.theta)):
            thetai = thetai-90
            print "(%d, %d)" %(ri, thetai)"""        
        print response
=======
	    count += 1
	    if count % 5 == 0:
		    print count    	
		    # response = sonar_image(start)
		    # server(start)    
		    #print "theta = %s, r = %s, status = %r" %(response.theta, response.r, response.status)
		    response = sonar_image(start)
		    print response
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
