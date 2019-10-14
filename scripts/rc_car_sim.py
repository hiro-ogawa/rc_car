#!/usr/bin/env python
import rospy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import (
    Pose2D,
    TransformStamped,
)
from ackermann_msgs.msg import AckermannDrive

# Because of transformations
from tf_conversions import transformations

WHEEL_BASE = 0.2

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

# if __name__ == '__main__':
#     rospy.Subscriber('/%s/pose' % turtlename,
#                      turtlesim.msg.Pose,
#                      handle_turtle_pose,
#                      turtlename)
#     rospy.spin()

def callback(data):
    drive = data.speed
    steer = data.steering_angle

    # servo_drive.ChangeDutyCycle(drive + 7.25)
    # servo_steer.ChangeDutyCycle(steer + 7.25)

def listener():
    rospy.init_node('ackmn_simulator', anonymous=True)
    rospy.Subscriber("/ackmn_drive", AckermannDrive, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
