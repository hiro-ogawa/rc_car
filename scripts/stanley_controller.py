#!/usr/bin/env python
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
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
max_steer = np.radians(10.0)  # [rad] max steering angle

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

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


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


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
   # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    closest_error = min(d)
    target_idx = d.index(closest_error)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      - np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


poses = []

def path_callback(msg):
    _poses = []
    for p in msg.poses:
        q = p.pose.orientation
        yaw = transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        _poses.append((p.pose.position.x, p.pose.position.y, yaw))

    global poses
    poses = _poses
    rospy.loginfo('next goal received { x: %s, y: %s, yaw: %s }', p.pose.position.x, p.pose.position.y, yaw)

if __name__ == '__main__':
    rospy.init_node('stanley_controller', anonymous=True)
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
        if not control:
            if poses:
                control = True
                vx = 0.0
                cx, cy, cyaw = zip(*poses)

                # Initial state
                try:
                    trans = tfBuffer.lookup_transform("world", 'estimated_footprint', rospy.Time(0))
                except (LookupException, ConnectivityException, ExtrapolationException):
                    rate.sleep()
                    continue

                q = trans.transform.rotation
                state = State(
                    x=trans.transform.translation.x,
                    y=trans.transform.translation.y,
                    yaw=transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2],
                    v=vx)

                last_idx = len(poses) - 1
                time = 0.0
                target_idx, _ = calc_target_index(state, cx, cy)
            else:
                rate.sleep()
                continue

        try:
            trans = tfBuffer.lookup_transform("world", 'estimated_footprint', rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            rate.sleep()
            continue
        q = trans.transform.rotation
        state = State(
            x=trans.transform.translation.x,
            y=trans.transform.translation.y,
            yaw=transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2],
            v=vx)

        vx += pid_control(target_speed, vx) * dt
        cx, cy, cyaw = zip(*poses)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)

        msg_ackmn = AckermannDrive()
        msg_ackmn.steering_angle = np.clip(di, -max_steer, max_steer)
        msg_ackmn.speed = vx
        pub_ackmn.publish(msg_ackmn)

        # if last_idx <= target_idx:
        #     control = False
        #     vx = 0.0
        #     poses = []

        #     msg_ackmn = AckermannDrive()
        #     msg_ackmn.steering_angle = 0
        #     msg_ackmn.speed = 0
        #     pub_ackmn.publish(msg_ackmn)

        rate.sleep()
