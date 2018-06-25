#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from zeabus_imaging_sonar.msg import sonar_msg
from zeabus_imaging_sonar.srv import sonar_srv
sonar_image = None

def server(start):
    global sonar_image
    # try:      
    response = sonar_image(start)
    print response
    return response.theta, response.r, response.status
    # except  rospy.ServiceException, e:
    #     print "Service call failed: %s"%e


if __name__ == "__main__":
    print "Waiting"
    start = True
    count = 0
    rospy.wait_for_service('/sonar_image')
    sonar_image = rospy.ServiceProxy('/sonar_image', sonar_srv)
    print 'service start'
    while not rospy.is_shutdown():
        count += 1
        time.sleep(1)
        response = sonar_image(start)
        """for i, (ri, thetai) in enumerate(zip(response.r, response.theta)):
            thetai = thetai-90
            print "(%d, %d)" %(ri, thetai)"""        
        print response
