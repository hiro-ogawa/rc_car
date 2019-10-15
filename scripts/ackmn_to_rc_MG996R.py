#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive

import RPi.GPIO as GPIO

PIN_DRIVE = 12
PIN_STEER = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_DRIVE, GPIO.OUT)
GPIO.setup(PIN_STEER, GPIO.OUT)

servo_drive = GPIO.PWM(PIN_DRIVE, 50)
servo_steer = GPIO.PWM(PIN_STEER, 50)

servo_drive.start(7.25)
servo_steer.start(6.5)

def callback(data):
    drive = data.speed
    steer = data.steering_angle

    servo_drive.ChangeDutyCycle(drive + 7.25)
    servo_steer.ChangeDutyCycle((steer*2)*-1 + 6.5)

def listener():
    rospy.init_node('rc_driver', anonymous=True)
    rospy.Subscriber("/ackmn_drive", AckermannDrive, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

    servo_drive.stop()
    servo_steer.stop()
    GPIO.cleanup()
