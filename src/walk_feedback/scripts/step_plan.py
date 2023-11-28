#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# step_plan
# generates position for the next step for THMOS
# created by qianruiluo(luoqr21@mails.tsinghua.edu.cn) on 2023/11/22
# last modified on 2023/11/22

import numpy as np
import imu_data

class step_plan():
    '''
    This is a class to generate the next-step position.

    '''
    def __init__(self, max_stride_x, max_stride_y, max_stride_th, period, width, accel_limit_coef=[0.3, 0.3, 0.3]): 
        ''' Initialize the step_plan class

        :param max_stride_x: Max single stride distance forward
        :param max_stride_y: Max single stride distance transverse
        :param max_stride_th: Max single turn angle
        :param period: preferred time to take one step
        :param width: distance between legs
        :param accel_limit_coef: limits max acceleration regarding max stride in [x, y, theta]

        '''
        # accel_limit_coef limits max acceleration regarding max stride
        self.accel_limit_x = accel_limit_coef[0] * period * max_stride_x
        self.accel_limit_y = accel_limit_coef[1] * period * max_stride_y
        self.accel_limit_th = accel_limit_coef[2] * period * max_stride_th
  
        # max speed of step 
        self.max_stride_x = max_stride_x
        self.max_stride_y = max_stride_y
        self.max_stride_th = max_stride_th

        # record last velocity
        self.last_vx = 0
        self.last_vy = 0
        self.last_vth = 0
    
        # step period
        self.period = period
        # distance between legs
        self.width = width

    def plan(self, goal_vx, goal_vy, goal_vth, next_support_leg, status = 'walking'):
        '''
        Returns the position of the next step (in the robot frame).
        Step plan should be conducted once the former step ends touching the ground.

        '''
        # limit speed
        goal_vx = max(-self.max_stride_x, min(goal_vx, self.max_stride_x))
        goal_vy = max(-self.max_stride_y, min(goal_vy, self.max_stride_y))
        goal_vth = max(-self.max_stride_th, min(goal_vth, self.max_stride_th))

        # limit acceleration
        dvx = min(self.accel_limit_x, abs(goal_vx - self.last_vx))
        if goal_vx < self.last_vx:
            dvx = -dvx
        dvy = min(self.accel_limit_y, abs(goal_vy - self.last_vy))
        if goal_vy < self.last_vy:
            dvy = -dvy
        dvth = min(self.accel_limit_th, abs(goal_vth - self.last_vth))
        if goal_vth < self.last_vth:
            dvth = -dvth
        goal_vx  = self.last_vx + dvx
        goal_vy  = self.last_vy + dvy
        goal_vth = self.last_vth + dvth

