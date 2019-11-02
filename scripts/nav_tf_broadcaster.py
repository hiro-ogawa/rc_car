#!/usr/bin/env python

import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
# Because of transformations
from tf_conversions import transformations

import pyproj

def fix_callback(msg):
    if msg.longitude is None:
        return

    rospy.loginfo('lat: %s, lon: %s', msg.latitude, msg.longitude)
    EPSG4612 = pyproj.Proj("+init=EPSG:4612")
    EPSG2451 = pyproj.Proj("+init=EPSG:2451")
    y,x = pyproj.transform(EPSG4612, EPSG2451, msg.longitude, msg.latitude)

    if np.isnan(x) or np.isnan(y):
        return

    tf = TransformStamped()
    tf.header.stamp = msg.header.stamp
    tf.header.frame_id = "world"
    tf.child_frame_id = "gps"
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = 0.0
    q = transformations.quaternion_from_euler(0, 0, 0)
    tf.transform.rotation.x = q[0]
    tf.transform.rotation.y = q[1]
    tf.transform.rotation.z = q[2]
    tf.transform.rotation.w = q[3]
    br.sendTransform(tf)

    rospy.loginfo("{}, {}".format(x, y))

if __name__ == '__main__':
    rospy.init_node('nav_tf_broadcaster', anonymous=True)
    rospy.Subscriber("/fix", NavSatFix, fix_callback)
    br = TransformBroadcaster()
    rospy.spin()
