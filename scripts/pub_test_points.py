#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose

ax = [0.0, 100.0, 100.0, 50.0, 60.0]
ay = [0.0, 0.0, -30.0, -20.0, 0.0]
points = zip(ax, ay)

if __name__ == '__main__':
    rospy.init_node('pub_test_points', anonymous=True)
    pub = rospy.Publisher("/path_points", PoseArray, latch=True)

    poses = PoseArray()

    poses.header.frame_id = "world"
    poses.header.stamp = rospy.Time.now()

    for p in points:
        print(p)
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.orientation.w = 1.0
        poses.poses.append(pose)

    print(poses)
    pub.publish(poses)

    rospy.spin()
