#!/usr/bin/env python

import sys
import cv2
import time
import numpy as np
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

count = 0

class main:
    def __init__(self):
        self.image = None
        self.subTopic = '/img_sonar/compressed'
        self.sub = rospy.Subscriber(self.subTopic, CompressedImage, self.callback,  queue_size = 1)
        
    def callback(self, ros_data):
        #print ros_data.data
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #print np_arr
        self.image = cv2.imdecode(np_arr, 1)
        #cv2.imshow("test",self.image)
        #print self.image

    def show(self):
        global count
        while(self.image is None):
            rospy.sleep(0.01)
        while not rospy.is_shutdown():
            print count
            count += 1
            cv2.imshow('sammy', self.image)
            cv2.waitKey(30)

if __name__ == '__main__':
    rospy.init_node('Tracking', anonymous=True)  
    try:
        count += 1
        preImg = main()
        preImg.show()
    except KeyboardInterrupt:
        print("Shutting down")