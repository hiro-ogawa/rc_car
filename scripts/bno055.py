#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Bosch BNO055 の Quaternion と TF を出力する
"""

from math import sqrt

import rospy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

from Adafruit_BNO055 import BNO055

if __name__ == '__main__':
    rospy.init_node('bno055_node')

    bno = BNO055.BNO055()
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    rospy.loginfo('System status: {0}'.format(status))
    rospy.loginfo('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        rospy.loginfo('System error: {0}'.format(error))
        rospy.loginfo('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    rospy.loginfo('Software version:   {0}'.format(sw))
    rospy.loginfo('Bootloader version: {0}'.format(bl))
    rospy.loginfo('Accelerometer ID:   0x{0:02X}'.format(accel))
    rospy.loginfo('Magnetometer ID:    0x{0:02X}'.format(mag))
    rospy.loginfo('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    br = TransformBroadcaster()
    quat_pub = rospy.Publisher('/imu_quat', Quaternion, queue_size=1)

    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        sys, gyro, accel, mag = bno.get_calibration_status()
        x, y, z, w = bno.read_quaternion()
        norm = sqrt(x**2 + y**2 + z**2 + w**2)

        q = Quaternion(x=x/norm, y=y/norm, z=z/norm, w=w/norm)
        quat_pub.publish(q)

        tf = TransformStamped()
        tf.header.stamp = rospy.get_rostime()
        tf.header.frame_id = "imu_footprint"
        tf.child_frame_id = "imu"
        tf.transform.rotation = q
        br.sendTransform(tf)

        rospy.logdebug_throttle(1, 'Q(xyzw)=({0:0.3F}, {1:0.3F}, {2:0.3F}, {3:0.3F}\tSys_cal={4} Gyro_cal={5} Accel_cal={6} Mag_cal={7}'.format(x, y, z, w, sys, gyro, accel, mag))
        rate.sleep()
