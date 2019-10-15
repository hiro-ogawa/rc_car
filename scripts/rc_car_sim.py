#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import (
    Pose2D,
    Pose,
    TransformStamped,
)
from ackermann_msgs.msg import AckermannDrive

# Because of transformations
from tf_conversions import transformations

class RCSim(object):
    def __init__(self, initial_pose=Pose2D(), wheel_base=0.2):
        self.pose = initial_pose
        self.pos = np.matrix([self.pose.x, self.pose.y, 1]).T
        self.dir = self.pose.theta

        self.wheel_base = wheel_base
        self.speed = 0.0
        self.steer = 0.0

    def ackmn_callback(self, msg):
        self.speed = msg.speed
        self.steer = msg.steering_angle

    def gen_rot(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        rot = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return rot

    def sim(self):
        t = rospy.get_rostime()
        dt = (t - self.t0).to_sec()
        self.t0 = t

        dx = self.speed * dt
        dy = 0.0

        trans = np.matrix([dx, dy, 1]).T

        hdtheta = 0
        if self.steer != 0.0:
            hdtheta = dx / (self.wheel_base / np.tan(self.steer)) / 2.0

        half_rot = self.gen_rot(hdtheta)
        pose_rot = self.gen_rot(self.dir)

        self.pos += pose_rot * half_rot * trans
        self.dir += hdtheta * 2.0

        self.pose.x = self.pos[0][0]
        self.pose.y = self.pos[1][0]
        self.pose.theta = self.dir

    def publish_msgs(self):
        t = rospy.Time.now()
        self.pub_pose2d.publish(self.pose)

        tf = TransformStamped()
        tf.header.stamp = t
        tf.header.frame_id = "world"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.pose.x
        tf.transform.translation.y = self.pose.y
        tf.transform.translation.z = 0.0
        q = transformations.quaternion_from_euler(0, 0, self.pose.theta)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.br.sendTransform(tf)

        tf.header.frame_id = "base_footprint"
        tf.child_frame_id = "steer"
        tf.transform.translation.x = self.wheel_base
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        q = transformations.quaternion_from_euler(0, 0, self.steer)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.br.sendTransform(tf)


    def run(self):
        rospy.init_node('ackmn_simulator', anonymous=True)
        rospy.Subscriber("/ackmn_drive", AckermannDrive, self.ackmn_callback)
        self.pub_pose2d = rospy.Publisher("/pose2d", Pose2D, queue_size=1)
        self.pub_pose = rospy.Publisher("/pose", Pose, queue_size=1)
        self.br = TransformBroadcaster()

        r = rospy.Rate(10)
        self.t0 = rospy.get_rostime()
        while not rospy.is_shutdown():
            self.sim()
            self.publish_msgs()
            r.sleep()


def handle_turtle_pose(msg, turtlename):
    br = TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    sim = RCSim()
    sim.run()
