#!/usr/bin/env python
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import math
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped
from ackermann_msgs.msg import AckermannDrive
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
    LookupException, ConnectivityException, ExtrapolationException,
)
# Because of transformations
from tf_conversions import transformations

k = 0.5  # control gain
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s] time difference
L = 0.2  # [m] Wheel base of vehicle
max_steer = np.radians(20.0)  # [rad] max steering angle

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, yaw):
    """
    Stanley steering control.

    :param state: (State object)
    :param yaw: float
    :return: float
    """
    # theta_e corrects the heading error
    print('current yaw', math.degrees(state.yaw), 'target yaw', math.degrees(yaw))
    theta_e = normalize_angle(yaw - state.yaw)
    # theta_d corrects the cross track error
    #theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e #+ theta_d
    #print('delta', math.degrees(delta))
    return delta


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


pose = None


def path_callback(msg):
    if len(msg.poses) == 0:
        return

    p = msg.poses[0]
    if p is None:
        return

    q = p.pose.orientation
    yaw = transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    x = p.pose.position.x
    y = p.pose.position.y
    rospy.loginfo('next goal received { x: %s, y: %s, yaw: %s }', x, y, yaw)

    global pose
    pose = (x, y, yaw)

if __name__ == '__main__':
    rospy.init_node('simple_controller', anonymous=True)
    rospy.Subscriber("/path", Path, path_callback)
    pub_ackmn = rospy.Publisher('/ackmn_drive', AckermannDrive, queue_size=1)
    tfBuffer = Buffer()
    listener = TransformListener(tfBuffer)
    br = TransformBroadcaster()
    target_speed = 1.0  # [m/s]

    control = False
    vx = 0.0

    rate = rospy.Rate(1.0 / dt)
    while not rospy.is_shutdown():
        # initialization
        if not control:
            if pose:
                control = True
                vx = 0.0
            else:
                rate.sleep()
                continue

        try:
            trans = tfBuffer.lookup_transform("world", 'estimated_footprint', rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rate.sleep()
            continue

        q = trans.transform.rotation
        x=trans.transform.translation.x
        y=trans.transform.translation.y

        current_tf = State(
            x=x, y=y, yaw=transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2],
            v=vx)

        vx += pid_control(target_speed, vx) * dt
        goal_x, goal_y, goal_yaw = pose
        yaw = math.atan2(goal_y-y, goal_x-x)  # yaw from current pos to the goal

        di = stanley_control(current_tf, yaw)  # adjust direction according to the current orientation of the car

        msg_ackmn = AckermannDrive()
        msg_ackmn.steering_angle = np.clip(di, -max_steer, max_steer)
        msg_ackmn.speed = vx
        pub_ackmn.publish(msg_ackmn)

        rate.sleep()

