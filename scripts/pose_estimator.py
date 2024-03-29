#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
GPS 情報を受信して 速度を計算
twist 出力から旋回量を計算 本番ではジャイロから出るはず
IMUから絶対角がそもそも出るんじゃないかな
"""

import numpy as np

import rospy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from tf2_msgs.msg import TFMessage
# Because of transformations
from tf_conversions import transformations

class PoseEstimator(object):
    last_pos = None
    pos = None
    dir = 0.0

    def twist_callback(self, msg):
        self.wz = msg.angular.z

    def tf_callback(self, msg):
        for tf in msg.transforms:
            if (tf.header.frame_id != "world") or (tf.child_frame_id != "gps"):
                return
            elif self.last_pos is None:
                self.last_pos = tf
                return

            dt = (tf.header.stamp - self.last_pos.header.stamp).to_sec()
            p0 = np.array([self.last_pos.transform.translation.x, self.last_pos.transform.translation.y])
            p1 = np.array([tf.transform.translation.x, tf.transform.translation.y])
            l = np.linalg.norm(p1 - p0)
            self.vx = l / dt

            self.pos = np.matrix([tf.transform.translation.x, tf.transform.translation.y, 1.0]).T
            q = tf.transform.rotation
            self.dir = transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

            self.last_pos = tf

    def gen_rot(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        rot = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return rot

    def sim(self):
        t = rospy.get_rostime()
        dt = (t - self.t0).to_sec()
        self.t0 = t

        dx = self.vx * dt
        dy = 0.0
        dtheta = self.wz * dt

        trans = np.matrix([dx, dy, 1]).T

        half_rot = self.gen_rot(dtheta / 2.0)
        pose_rot = self.gen_rot(self.dir)

        self.pos += pose_rot * half_rot * trans
        self.dir += dtheta

    def run(self):
        rospy.init_node('pose_estimator')
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/twist_sim", Twist, self.twist_callback)

        br = TransformBroadcaster()

        rate = rospy.Rate(50.0)
        self.t0 = rospy.get_rostime()
        while not rospy.is_shutdown():
            rate.sleep()

            if self.pos is not None:
                self.sim()
                tf = TransformStamped()
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = "world"
                tf.child_frame_id = "estimated_footprint"
                tf.transform.translation.x = self.pos[0][0]
                tf.transform.translation.y = self.pos[1][0]
                tf.transform.translation.z = 0.0
                q = transformations.quaternion_from_euler(0, 0, self.dir)
                tf.transform.rotation.x = q[0]
                tf.transform.rotation.y = q[1]
                tf.transform.rotation.z = q[2]
                tf.transform.rotation.w = q[3]
                br.sendTransform(tf)

if __name__ == '__main__':
    estimator = PoseEstimator()
    estimator.run()
