#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# created by qianruiluo(luoqr21@mails.tsinghua.edu.cn) on    2023/11/1
# last modified 2023/11/22

import time
import math
import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist


class imu_Subscriber(object):
    def __init__(self):
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.velo_sub = rospy.Subscriber('/system_speed', Twist, self.velo_callback)

        self.imu_ready = False
        self.velo_ready = False

        # robot orientation
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        # acceleration
        self.ax = 0
        self.ay = 0
        self.az = 0
        #body velocity
        self.vx = 0
        self.vy = 0
        self.vz = 0
        #angular velocity
        self.wx = 0
        self.wy = 0
        self.wz = 0
        #angular acceleration
        self.bx = 0
        self.by = 0
        self.bz = 0


    # imu topic contains data of orientation, acceleration and angular velocity.
    def imu_callback(self, imu_msg: Imu):
        self.quaternion_to_rpy(
            [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
        # self.orientation = self.yaw - self.theta
        # self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi

        self.ax = imu_msg.linear_acceleration.x
        self.ay = imu_msg.linear_acceleration.y
        self.az = imu_msg.linear_acceleration.z

        wx_new = imu_msg.angular_velocity.x
        wy_new = imu_msg.angular_velocity.y
        wz_new = imu_msg.angular_velocity.z 

        self.bx = (wx_new - self.wx) * 100
        self.by = (wy_new - self.wy) * 100
        self.bz = (wz_new - self.wz) * 100

        self.wx = wx_new
        self.wy = wy_new
        self.wz = wz_new
       
        self.imu_ready = True

    # system_speed topic contains data of body velocity.
    def velo_callback(self, velo_msg: Twist):
        self.vx = velo_msg.linear.x
        self.vy = velo_msg.linear.y
        self.vz = velo_msg.linear.z

        self.velo_ready = True

    def quaternion_to_rpy(self, quaternion):
        # 提取四元数的x、y、z、w分量
        x, y, z, w = quaternion
        quaternion_msg = Float32MultiArray()
        quaternion_msg.data = [x, y, z, w]

        # 计算旋转矩阵
        rotation_matrix = np.array([[1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                                    [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                                    [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]])

        # 提取RPY角
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        # 前x右y下z,如果参考系改的话改这里
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

if __name__ == '__main__':
    rospy.init_node('thmos_walk_imu', anonymous=True)
    imu = imu_Subscriber()
    while True:
        if imu.imu_ready:
            print("a:")
            print(imu.ax, imu.ay, imu.az)
            print("w:")
            print(imu.wx, imu.wy, imu.wz)
            print("b:")
            print(imu.bx, imu.by, imu.bz)
            imu.imu_ready = False
        if imu.velo_ready:
            print("v:")
            print(imu.vx, imu.vy, imu.vz)
            imu.velo_ready = False
