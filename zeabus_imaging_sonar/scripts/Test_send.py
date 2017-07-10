#!/usr/bin/env python

import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

if __name__ == '__main__':

    rospy.init_node('VideoPublisher', anonymous=True)
    pub = rospy.Publisher('/img_sonar/compressed', CompressedImage, queue_size=1)
    count = 0

    while not rospy.is_shutdown() :
        count += 1
        frame = cv2.imread('/home/johny/Desktop/sonar/img_'+str(count)+'.jpg')
        if frame is None:
            continue
        print(type(frame))
        frame = np.array(frame,np.uint8)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.header.stamp = rospy.Time.now()
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        #print msg.data
        pub.publish(msg)
        print "Sent!!"
        cv2.imshow('img',frame)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            break