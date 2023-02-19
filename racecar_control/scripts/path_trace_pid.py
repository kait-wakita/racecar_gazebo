#!/usr/bin/env python
# -*- coding: utf-8 -*-
#########################################################################
# 車線トレース
#     v1.0   2023/02/19      
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
from ackermann_msgs.msg import AckermannDriveStamped
from scipy.spatial import KDTree

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


#########################################
# 補助関数
#########################################
class PathTrace(object):
    def __init__(self):
        self._pose_sub = rospy.Subscriber('egocar_pose', Pose, self.callback)
        self._velocity_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.iframe = 0

        path_raw = np.loadtxt('centerline_fix.csv', delimiter=',', dtype='int64')
        path_xy = [[0,0]]
        for pxy in path_raw:
            x,y = pix2xy(pxy[0],pxy[1])
            path_xy=np.vstack([path_xy,np.array((x,y))])
        self.path_xy = path_xy
        self.kd_tree = KDTree(path_xy)   # 最近傍点探索のためのkd-tree作成


    def callback(self, data):
        ####################################
        # トピックegocar_poseから姿勢取得
        ####################################
        x_w = data.position.x
        y_w = data.position.y
        q = data.orientation
        e = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
        theta_wr = e[2]

        #########################################
        # 目標とするのコース近傍点探索、座標変換
        #########################################
        v = 1.0
        lookahead_time = 0.5
        x_l = x_w + v * lookahead_time * math.cos(theta_wr)
        y_l = y_w + v * lookahead_time * math.sin(theta_wr)

        dist, index = self.kd_tree.query(np.array((x_l,y_l)))
        x_t = self.path_xy[index,0]
        y_t = self.path_xy[index,1]
        A = np.array([[math.cos(theta_wr),math.sin(theta_wr)],[-math.sin(theta_wr),math.cos(theta_wr)]])
        target_vcs = np.dot(A,np.array((x_t-x_w,y_t-y_w)))

        #########################################
        # 車両制御
        #########################################
        wr = math.atan2(target_vcs[1],target_vcs[0]) / lookahead_time
        wr = max(min(wr,0.8),-0.8)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.speed = v
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = wr
        msg.drive.steering_angle_velocity = 1

        self._velocity_pub.publish(msg) 
                           

        #rospy.loginfo("target = x:%5.2f(%5.2f), y:%5.2f(%5.2f)" % (x_t, x_w, y_t, y_w))
        rospy.loginfo("vcs=x:%5.2f, y:%5.2f, w=%5.2f" % (target_vcs[0], target_vcs[1], wr))

        self.iframe = self.iframe + 1


if __name__ == '__main__':
    rospy.init_node('path_trace')
    color = PathTrace()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

