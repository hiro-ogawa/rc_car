#!/usr/bin/env python
import time
import rospy
from ackermann_msgs.msg import AckermannDrive

import Adafruit_PCA9685

CHANNEL_STEER = 0
CHANNEL_DRIVE = 1

# Servo pulse lengths (out of 4096)
STEER_MIN = 230
STEER_MAX = 520
STEER_RANGE = STEER_MAX - STEER_MIN
STEER_NEUTRAL = STEER_RANGE // 2 + STEER_MIN
DRIVE_MIN = 250
DRIVE_MAX = 500
DRIVE_RANGE = DRIVE_MAX - DRIVE_MIN
DRIVE_NEUTRAL = DRIVE_RANGE // 2 + DRIVE_MIN

pwm = Adafruit_PCA9685.PCA9685()  # default i2c address: 0x40
pwm.set_pwm_freq(60)  # Hz


def callback(data):
    drive = data.speed
    steer = data.steering_angle

    pwm.set_pwm(CHANNEL_STEER, 0, STEER_NEUTRAL - int(steer * STEER_RANGE)//2)
    pwm.set_pwm(CHANNEL_DRIVE, 0, DRIVE_NEUTRAL + int(drive * DRIVE_RANGE)//2)

def listener():
    rospy.init_node('rc_driver', anonymous=True)
    rospy.Subscriber("/ackmn_drive", AckermannDrive, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
