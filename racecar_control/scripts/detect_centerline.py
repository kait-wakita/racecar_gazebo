#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class ColorExtract(object):
    def __init__(self):
        self._blue_pub = rospy.Publisher('blue_image', Image, queue_size=1)
        self._yellow_pub = rospy.Publisher('yellow_image', Image, queue_size=1)
        self._image_sub = rospy.Subscriber('/webcam_0_camera/image_raw', Image, self.callback)
        self._bridge = CvBridge()
        self._vel = Twist()

    def get_colored_area(self, cv_image, lower, upper):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return (area, extracted_image)
        
    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        blue_area, blue_image = self.get_colored_area(
            cv_image, np.array([150,0,0]), np.array([255,50,50]))
        yellow_area, yellow_image = self.get_colored_area(
            cv_image, np.array([0,150,150]), np.array([50,255,255]))
            
        try:
            self._blue_pub.publish(self._bridge.cv2_to_imgmsg(blue_image, 'bgr8'))
            self._yellow_pub.publish(self._bridge.cv2_to_imgmsg(yellow_image, 'bgr8'))
        except CvBridgeError, e:
            print e
        rospy.loginfo('blue=%d, yellow=%d' % (blue_area, yellow_area))


if __name__ == '__main__':
    rospy.init_node('detect_centerline')
    color = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

