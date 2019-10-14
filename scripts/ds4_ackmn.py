#!/usr/bin/env python
import rospy
from ds4_driver.msg import Status
from ackermann_msgs.msg import AckermannDrive

def callback(data):
    msg = AckermannDrive()
    msg.steering_angle = data.axis_right_x
    msg.speed = data.axis_left_y
    # print(msg)
    pub.publish(msg)

def listener():
    global pub
    rospy.init_node('ds4_ackmn', anonymous=True)
    rospy.Subscriber("/status", Status, callback)
    pub = rospy.Publisher('/ackmn_drive', AckermannDrive, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()