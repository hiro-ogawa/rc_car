#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

def callback(data):
    msg = AckermannDrive()
    msg.steering_angle = data.angular.z / 4.0
    msg.speed = data.linear.x
    # print(msg)
    pub.publish(msg)

def listener():
    global pub
    rospy.init_node('twist_to_ackmn', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    pub = rospy.Publisher('/ackmn_drive', AckermannDrive, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
