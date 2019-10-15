#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
1秒に1回 world -> base_footprint の tf を取得して
pose と tf を吐く
"""

import rospy

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
    LookupException, ConnectivityException, ExtrapolationException,
)
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('dummy_gps_broadcaster')

    tfBuffer = Buffer()
    listener = TransformListener(tfBuffer)
    br = TransformBroadcaster()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("world", 'base_footprint', rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rate.sleep()
            continue

        tf = TransformStamped()
        tf.header.stamp = rospy.get_rostime()
        tf.header.frame_id = "world"
        tf.child_frame_id = "gps"
        tf.transform = trans.transform
        br.sendTransform(tf)

        rate.sleep()