#!/usr/bin/env python
# -*- coding: utf-8 -*-
#########################################################################
# 天井カメラを用いた車両制御
#     v1.0   2023/02/19      作成、方向推定方法は暫定
#########################################################################

import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import tf
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError

#########################################
# 補助関数
#########################################
def pix2xy(px,py):
    x = (px-1423)*4.35/996
    y = (1041-py)*4.35/996
    return x,y

def regulate_degree(d):
    dd = (d+180) % 360.0
    if dd < 0:
        dd = dd + 360
    return dd-180

class CarExtract(object):
    def __init__(self):
        self._image_sub = rospy.Subscriber('/webcam0/image_raw', Image, self.callback)
        self._bbox_pub = rospy.Publisher('bbox_image', Image, queue_size=1)
        self._bridge = CvBridge()
        self._pose_pub = rospy.Publisher('egocar_pose', Pose, queue_size=1)
        self._pose = Pose()
        self.model = cv2.bgsegm.createBackgroundSubtractorMOG()
        self.iframe = 0
        self.theta_prev = 0
        self.theta_base = 0
 
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

        # 輪郭抽出する (opencv3.0用は[1]、4.0以降は[0])。
        mask = self.model.apply(cv_image)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        if contours:
            # 最大輪郭（車両）の抽出、2台以上の場合要改良
            max_cnt = max(contours, key=lambda x: cv2.contourArea(x))
            max_cnt_area = cv2.contourArea(max_cnt)

            if max_cnt_area > 5000:
                # 輪郭を囲む外接矩形を取得する。
                rect = cv2.minAreaRect(max_cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                frame = cv2.drawContours(cv_image,[box], 0,(0, 255, 0), 2)

                # 角度正規化
                if self.iframe < 10:
                    theta = self.theta_prev
                elif rect[1][0] < rect[1][1]:
                    theta = self.theta_base + rect[2] + 90
                else:
                    theta = self.theta_base + rect[2] + 0           
                if theta - self.theta_prev < -90:
                    theta = theta + 180
                    self.theta_base = self.theta_base + 180
                elif theta - self.theta_prev > 90:
                    theta = theta - 180
                    self.theta_base = self.theta_base - 180
                self.theta_prev = theta
                theta_w = regulate_degree(-theta+180)
                x_w,y_w =  pix2xy(rect[0][0],rect[0][1])

                # rosへのpublish
                self._pose.position.x = x_w
                self._pose.position.y = y_w
                q = tf.transformations.quaternion_from_euler(0, 0, math.radians(theta_w))
                self._pose.orientation.x = q[0]
                self._pose.orientation.y = q[1]
                self._pose.orientation.z = q[2]
                self._pose.orientation.w = q[3]

                br = tf.TransformBroadcaster()
                br.sendTransform((x_w,y_w,0),q,rospy.Time.now(),"egocar","webcam0")
                           
                try:
                    self._bbox_pub.publish(self._bridge.cv2_to_imgmsg(frame, 'bgr8'))
                    self._pose_pub.publish(self._pose)
                except CvBridgeError, e:
                    print e
                rospy.loginfo("x:%4.1f, y:%4.1f, theta:%4.1f, area:%d" % (x_w,y_w, theta_w, cv2.contourArea(max_cnt)))

        self.iframe = self.iframe + 1


if __name__ == '__main__':
    rospy.init_node('carpose_extract')
    color = CarExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

