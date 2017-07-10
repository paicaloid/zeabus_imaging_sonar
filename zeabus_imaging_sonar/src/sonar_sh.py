#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

image = None

def callback1(ros_data):
    global image
    width = 640
    height = 512
    np_arr = np.fromstring(ros_data.data, np.uint8)
    # bridge = CvBridge()
    #image = cv2.resize(cv2.imdecode(
    #    np_arr, 1), (width, height))
    image = cv2.imdecode(np_arr, 1)


if __name__ == '__main__':
    rospy.init_node('SonarShowImage', anonymous=True)
    sub = rospy.Subscriber('/image/Tracking', CompressedImage,
                           callback1, queue_size=10)

    while not rospy.is_shutdown():
        if image is None:
            continue

        cv2.imshow('image', image.copy())
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            break
        rospy.sleep(0.1)
cv2.destroyAllWindows()