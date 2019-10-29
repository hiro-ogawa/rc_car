#!/usr/bin/env python
import copy

import rospy
from geometry_msgs.msg import PoseArray, Pose, Point
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

# ax = [0.0, 10.0, 10.0, 5.0, 6.0]
# ay = [0.0, 0.0, -3.0, -2.0, 0.0]
ax = [0.0, 1.0, 2.0, 3.0, 6.0,  3.0, -2.0, -5.0, -2.0, 0.0]
ay = [3.0, 1.0, 2.0, 3.0, 0.0, -3.0, -3.0,  0.0,  3.0, 3.0]

points = zip(ax, ay)

def processFeedback( feedback ):
    server.applyChanges()

    poses = PoseArray()
    poses.header.frame_id = "world"
    poses.header.stamp = rospy.Time.now()
    for i in range(len(server.marker_contexts)):
        name = "p{}".format(i)
        p = server.get(name).pose.position
        pose = Pose()
        pose.position.x = p.x
        pose.position.y = p.y
        pose.orientation.w = 1.0
        poses.poses.append(pose)
    # print(poses)
    pub_posess.publish(poses)


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

def makeChessPieceMarker(position, name="", description=""):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position = position
    int_marker.scale = 0.5

    int_marker.name = name
    int_marker.description = description

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = -1
    control.orientation.z = 0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append( makeBox(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, processFeedback)

if __name__ == '__main__':
    rospy.init_node('path_points_publisher', anonymous=True)
    pub_posess = rospy.Publisher("/path_points", PoseArray, latch=True)
    server = InteractiveMarkerServer("path_points_publisher")

    poses = PoseArray()

    poses.header.frame_id = "world"
    poses.header.stamp = rospy.Time.now()

    for i, p in enumerate(points):
        makeChessPieceMarker(Point(p[0], p[1], 0), "p{}".format(i))

        # print(p)
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.orientation.w = 1.0
        poses.poses.append(pose)

    server.applyChanges()

    # print(poses)
    pub_posess.publish(poses)

    rospy.spin()
