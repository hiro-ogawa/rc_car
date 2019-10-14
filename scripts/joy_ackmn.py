#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

def callback(data):
    msg = AckermannDrive()
    msg.steering_angle = data.axes[2]
    msg.speed = data.axes[1]
    # print(msg)
    pub.publish(msg)

def listener():
    global pub
    rospy.init_node('ds4_ackmn', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    pub = rospy.Publisher('/ackmn_drive', AckermannDrive, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()