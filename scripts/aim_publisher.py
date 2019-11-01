#!/usr/bin/env python

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
    LookupException, ConnectivityException, ExtrapolationException,
)
from tf_conversions import transformations

ax = [5.0, 10.0, 5.0, 0.0, -5.0, -10.0, -5.0]
ay = [5.0, 0.0, -5.0, 0.0,  5.0,   0.0, -5.0]
points = zip(ax, ay)

def publish_point(pub_path, point, yaw):
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()

    pose = PoseStamped()
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    q = transformations.quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    path.poses.append(pose)
    rospy.loginfo('publish next goal { x: %s, y: %s, yaw: %s', point[0], point[1], yaw)
    pub_path.publish(path)

if __name__ == '__main__':
    rospy.init_node('aim_publisher', anonymous=True)
    tfBuffer = Buffer()
    listener = TransformListener(tfBuffer)
    br = TransformBroadcaster()
    pub_path = rospy.Publisher("/path", Path, queue_size=1)
    i = -1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("world", 'estimated_footprint', rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rate.sleep()
            continue

        q = trans.transform.rotation
        x=trans.transform.translation.x
        y=trans.transform.translation.y
        yaw_base=transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        if i == -1:
            i = 0
            yaw = math.atan2(points[i][1] - y,points[i][0] - x)
            publish_point(pub_path, points[i], yaw)

        d = np.sqrt((x - points[i][0]) ** 2 + (y - points[i][1]) ** 2)
        rospy.loginfo("distance to point %s: %s", i, d)

        if d < 3:
            print('arrived at point ', i)
            i += 1
            if i == len(points):
                print('lap done')
                i = 0
            yaw = math.atan2(points[i][1] - y,points[i][0] - x)
            publish_point(pub_path, points[i], yaw)

        rate.sleep()
