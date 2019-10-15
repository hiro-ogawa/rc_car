#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from ackermann_msgs.msg import AckermannDrive

# Because of transformations
from tf_conversions import transformations

class RCSim(object):
    def __init__(self, initial_pose=(0, 0, 0), wheel_base=0.2):
        self.pos = np.matrix([float(initial_pose[0]), float(initial_pose[1]), 1.0]).T
        self.dir = float(initial_pose[2])

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

        self.twist = Twist()
        self.twist.linear.x = self.speed
        self.twist.angular.z = self.speed * np.tan(self.steer) / self.wheel_base

        dx = self.twist.linear.x * dt
        dy = 0.0

        trans = np.matrix([dx, dy, 1]).T

        hdtheta = 0
        if self.steer != 0.0:
            hdtheta = self.twist.angular.z * dt / 2.0

        half_rot = self.gen_rot(hdtheta)
        pose_rot = self.gen_rot(self.dir)

        self.pos += pose_rot * half_rot * trans
        self.dir += hdtheta * 2.0

    def publish_msgs(self):
        t = rospy.Time.now()

        tf = TransformStamped()
        tf.header.stamp = t
        tf.header.frame_id = "world"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.pos[0][0]
        tf.transform.translation.y = self.pos[1][0]
        tf.transform.translation.z = 0.0
        q = transformations.quaternion_from_euler(0, 0, self.dir)
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

        self.pub_twist.publish(self.twist)

    def run(self):
        rospy.init_node('ackmn_simulator', anonymous=True)
        rospy.Subscriber("/ackmn_drive", AckermannDrive, self.ackmn_callback)
        self.pub_twist = rospy.Publisher("/twist_sim", Twist, queue_size=1)
        self.br = TransformBroadcaster()

        r = rospy.Rate(50)
        self.t0 = rospy.get_rostime()
        while not rospy.is_shutdown():
            self.sim()
            self.publish_msgs()
            r.sleep()


if __name__ == '__main__':
    sim = RCSim()
    sim.run()
