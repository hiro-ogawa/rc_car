#!/usr/bin/env python
import math
import time

import rospy
from ackermann_msgs.msg import AckermannDrive

import Adafruit_PCA9685

CHANNEL_STEER = 0
CHANNEL_DRIVE = 1

STEER_MIN = 230
STEER_MAX = 430
STEER_RANGE = STEER_MAX - STEER_MIN
STEER_NEUTRAL = 340

MAX_STEER_ANGLE = math.radians(20)


DRIVE_NEUTRAL = 375

DRIVE_BACK_MAX = 250 
DRIVE_BACK_RANGE = DRIVE_NEUTRAL - DRIVE_BACK_MAX

DRIVE_FWRD_MAX = 450
DRIVE_FWRD_RANGE = DRIVE_FWRD_MAX - DRIVE_NEUTRAL

# empirical max speed on DRIVE_FWRD_MAX
MAX_SPEED = 8.3  # m/s = 30km/h  

SPEED_LIMI = MAX_SPEED  # no limit if max speed

pwm = Adafruit_PCA9685.PCA9685()  # default i2c address: 0x40
pwm.set_pwm_freq(60)  # Hz

JOY = False  # if True, input value is within the range of -1.0 - 1.0


def normalize_drive(speed):
    """ convert speed m/s to the range from 0 to 1.0
    """
    return speed/MAX_SPEED

def normalize_steer(angle_rad):
    """ convert speed m/s to the range from 0 to 1.0
    """
    return angle_rad/MAX_STEER_ANGLE

def to_pwm(drive, steer):
    print('drive %s, steer=%s', drive, math.degrees(steer))

    #if drive > 0.15:
    #    drive = 0.15
    if not JOY:
        drive = normalize_drive(drive)
        steer = normalize_steer(steer)

    steer_pwm = STEER_NEUTRAL - int(steer * STEER_RANGE)//2

    if drive > 0:
        drive_pwm = DRIVE_NEUTRAL + int(drive * DRIVE_FWRD_RANGE)
    else: 
        drive_pwm = DRIVE_NEUTRAL - int(abs(drive) * DRIVE_BACK_RANGE)

    if steer_pwm > STEER_MAX:
        steer_pwm = STEER_MAX
    if steer_pwm < STEER_MIN:
        steer_pwm = STEER_MIN
    
    #print('drive pwm', drive_pwm)
    #print('steer pwm', steer_pwm)
    return drive_pwm, steer_pwm

def callback(data):
    drive_pwm, steer_pwm = to_pwm(data.speed, data.steering_angle)

    pwm.set_pwm(CHANNEL_DRIVE, 0, drive_pwm)
    pwm.set_pwm(CHANNEL_STEER, 0, steer_pwm)

def listener():
    rospy.init_node('rc_driver', anonymous=True)
    rospy.Subscriber("/ackmn_drive", AckermannDrive, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    finally:
        pwm.set_pwm(CHANNEL_DRIVE, 0, DRIVE_NEUTRAL)
