#!/usr/bin/env python
import rospy
import json
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, TransformStamped, Twist
from tf2_ros import TransformBroadcaster

import pyproj

if __name__ == '__main__':
    rospy.init_node('pub_marker_array', anonymous=True)
    pub_markers = rospy.Publisher("/markers", MarkerArray, queue_size=1, latch=True)
    br = TransformBroadcaster()

    markers = MarkerArray()
    map_tf = TransformStamped()

    EPSG4612 = pyproj.Proj("+init=EPSG:4612")
    EPSG2451 = pyproj.Proj("+init=EPSG:2451")

    with open("/home/ogawa/catkin_ws/src/rc_car/data/points_testrun2.json") as f:
        points = json.load(f)

    id = 0
    for k, v in points.items():
        y, x = pyproj.transform(EPSG4612, EPSG2451, v["lon"], v["lat"])

        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.CYLINDER
        marker.scale = Vector3(x=0.1, y=0.1, z=0.5)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.ns = k
        marker.id = id
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = marker.scale.z / 2.0
        marker.pose.orientation.w = 1.0
        markers.markers.append(marker)

        id += 1

        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale = Vector3(z=0.5)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.ns = k + "_text"
        marker.id = id
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = marker.scale.z * 1.4
        marker.pose.orientation.w = 1.0
        marker.text = k
        markers.markers.append(marker)

        id += 1

        if k == "WP3":
            map_tf.header.frame_id = "world"
            map_tf.child_frame_id = "map"
            map_tf.transform.translation.x = x
            map_tf.transform.translation.y = y
            map_tf.transform.rotation.w = 1.0

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        for m in markers.markers:
            m.header.stamp = t
        map_tf.header.stamp = t

        pub_markers.publish(markers)
        br.sendTransform(map_tf)
        rate.sleep()
