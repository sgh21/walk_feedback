#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# original author: citbrains 
# generating walking pattern for the Thmos
# ver: 20230805

import math
import numpy as np
from thmos_kinematics import *
from foot_step_planner import *
from preview_control import *


class walking():
  def __init__(self, **WalkParams):

    # load params
    for key, value in WalkParams.items():
        setattr(self, key, value)

    # init packet
    self.kine = THMOSLegIK(self.way_left, self.way_right, self.leg_rod_length)
    self.pc = preview_control(self.dt, self.optimize_period, self.com_height)
    self.fsp = foot_step_planner(self.max_vx, self.max_vy, self.max_vth, self.walking_period, self.foot_width, self.plan_step_num)

    # init status 
    self.status = 'start'
    self.next_leg = 'both'
    
    self.X = np.matrix([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
    self.left_up = self.right_up = 0.0
    self.th = 0
    self.foot_h = self.foot_height
    self.old_vel = [0, 0, 0]
    self.both_foot_support_frames = self.both_foot_support_time / self.dt
    self.one_step_frames = self.walking_period / self.dt

    # leg offset
    self.left_off,  self.left_off_g,  self.left_off_d  = np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]) 
    self.right_off, self.right_off_g, self.right_off_d = np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]])

    # min foot step len
    self.min_foot_step_len = math.ceil(self.optimize_period / self.walking_period) + 2
    
    # step status
    self.foot_step = []
    self.foot_step_old = []
    self.foot_step_now = []
    self.foot_step_new = []
    
    # pattern
    self.pattern = []
    self.pattern_old = []
    self.pattern_now = []
    self.pattern_new_old = []
    self.pattern_new_now = []
    
    self.setGoalVel()
    return

  def setGoalVel(self, vel = None):
    '''get vel command and be a state machine'''
    if ((vel == None or (self.old_vel[0] == vel[0] and self.old_vel[1] == vel[1] and self.old_vel[2] == vel[2])) and len(self.foot_step) > self.min_foot_step_len):
      del self.foot_step[0]
    else:
      if len(self.foot_step) > 2:
        
        #com offset
        if self.next_leg == 'left':
          offset_y = -self.foot_width
        elif self.next_leg == 'right':
          offset_y = self.foot_width
        else:
          offset_y = 0.0
        current_x, current_y, current_th = self.foot_step[1][1], self.foot_step[1][2]+offset_y, self.foot_step[1][3]
      else:
      
        # start status
        current_x, current_y, current_th = 0, 0, 0
      
      # caculate new step plan
      if (vel == None):
        self.foot_step = self.fsp.calculate(self.old_vel[0], self.old_vel[1], self.old_vel[2], current_x, current_y, current_th, self.next_leg, self.status)
      else:
        self.foot_step = self.fsp.calculate(vel[0], vel[1], vel[2], current_x, current_y, current_th, self.next_leg, self.status)
        self.old_vel = vel.copy()

    self.status = 'walking'
    # caculate walk steps from Com trajectory
    t = self.foot_step[0][0]
    self.pattern, x, y = self.pc.set_param(t, self.X[:,0], self.X[:,1], self.foot_step)
    self.X = np.matrix([[x[0,0], y[0,0]], [x[1,0], y[1,0]], [x[2,0], y[2,0]]])
    
    if self.foot_step[0][4] == 'left':
      if  self.foot_step[1][4] == 'both': # for stop walk
        self.next_leg = 'both'
      else:
        self.next_leg = 'right'
      
    if self.foot_step[0][4] == 'right':
      if  self.foot_step[1][4] == 'both': # for stop walk
        self.next_leg = 'both'
      else:
        self.next_leg = 'left'

    if self.foot_step[0][4] == 'both':
      self.next_leg = 'both'
      
    self.foot_step_new_old, self.foot_step_new_now = self.foot_step[0].copy(), self.foot_step[1].copy()
    self.pattern_now = self.pattern.copy()
    return 
    
  def updateWalkGoal(self):
    '''get now walking goal'''
    # new params
    self.foot_step_old = self.foot_step_new_old.copy()
    self.foot_step_now = self.foot_step_new_now.copy()
    self.pattern_old = self.pattern_now.copy()
    
    if self.foot_step_old[4] == 'left':
      # leg next target
      if  self.foot_step_now[4] == 'both': # for stop walk
        self.right_off_g = np.matrix([[self.foot_step_now[1], self.foot_step_now[2], self.foot_step_now[3]]])
      else:
        self.right_off_g = np.matrix([[self.foot_step_now[1], self.foot_step_now[2] + self.foot_width, self.foot_step_now[3]]])
      self.right_off_d = (self.right_off_g - self.right_off) / (self.one_step_frames - self.both_foot_support_frames)
      
    if self.foot_step_old[4] == 'right':
      # leg next target
      if  self.foot_step_now[4] == 'both': # for stop walk
        self.left_off_g  = np.matrix([[self.foot_step_now[1], self.foot_step_now[2], self.foot_step_now[3]]])
      else:
        self.left_off_g  = np.matrix([[self.foot_step_now[1], self.foot_step_now[2] - self.foot_width, self.foot_step_now[3]]])
      self.left_off_d  = (self.left_off_g - self.left_off) / (self.one_step_frames - self.both_foot_support_frames)

    if self.foot_step_old[4] == 'both':   
      # leg next target
      self.left_off_g  = np.matrix([[self.foot_step_now[1], self.foot_step_now[2], self.foot_step_now[3]]])
      
      
  def getNextPos(self):
    '''set each movement'''
    # type keep
    self.left_off = self.left_off.astype(np.float64)
    self.left_off_d = self.left_off_d.astype(np.float64)
    self.right_off = self.right_off.astype(np.float64)
    self.right_off_d = self.right_off_d.astype(np.float64)

    # init
    X = self.pattern_old.pop(0)
    
    # period
    period = round(self.one_step_frames)
    start_up = round(self.both_foot_support_frames)
    end_up   = period / 2
    period_up = end_up - start_up
    
    # make drop slowly
    period_do = period - end_up - start_up
    
    # caculate offset and foot height
    if self.foot_step_old[4] == 'right':

      # up or down foot
      if start_up < (period-len(self.pattern_old)) <= end_up:
        self.left_up  += self.foot_h/period_up
      elif self.left_up > 0:
        self.left_up  = max(self.left_up  - self.foot_h/period_do, 0.0)

      # move foot in the axes of x,y,theta
      if (period-len(self.pattern_old)) > start_up:
        self.left_off += self.left_off_d
        if (period-len(self.pattern)) > (start_up + period_up * 2):
          self.left_off = self.left_off_g.copy()

    if self.foot_step_old[4] == 'left':

      # up or down foot
      if start_up < (period-len(self.pattern_old)) <= end_up:
        self.right_up += self.foot_h/period_up
      elif self.right_up > 0:
        self.right_up = max(self.right_up - self.foot_h/period_do, 0.0)

      # move foot in the axes of x,y,theta
      if (period-len(self.pattern_old)) > start_up:
        self.right_off += self.right_off_d
        if (period-len(self.pattern)) > (start_up + period_up * 2):
          self.right_off = self.right_off_g.copy()

    self.th = (self.foot_step_now[3]-self.foot_step_old[3])/period * (period-len(self.pattern_old)) + self.foot_step_old[3]
    
    # caculate foot pos
    lo = self.left_off  - np.block([[X[0,0:2],0]])
    ro = self.right_off - np.block([[X[0,0:2],0]])
    
    left_foot  = [lo[0,0] + self.com_x_offset, lo[0,1] + self.com_y_offset + self.ex_foot_width, self.left_up-  self.full_leg_length, 0.0, 0.0, self.th-lo[0,2]]
    right_foot = [ro[0,0] + self.com_x_offset, ro[0,1] + self.com_y_offset - self.ex_foot_width, self.right_up- self.full_leg_length, 0.0, 0.0, self.th-ro[0,2]]

    l_joint_angles = self.kine.LegIKMove('left',left_foot)
    r_joint_angles = self.kine.LegIKMove('right',right_foot)
    self.joint_angles = r_joint_angles+l_joint_angles  # R first

    xp = [X[0,2], X[0,3]]

    return self.joint_angles, left_foot, right_foot, xp, len(self.pattern_old)

if __name__ == '__main__':
  Params = {              
  'foot_width' : 0.06,
  'ex_foot_width' : 0.0,
  'foot_height' : 0.06,
  'com_height' : 0.3,
  'optimize_period' : 1.0,
  'walking_period' : 0.3,
  'both_foot_support_time' : 0.08,
  'dt' : 0.01,
  'max_vx' : 0.05,
  'max_vy': 0.03,
  'max_vth' : 0.2,
  'full_leg_length' : 0.3,
  'plan_step_num' : 20,
  'com_x_offset' : 0.0,
  'com_y_offset' : 0.0,
  'way_left' : [1,-1,-1,-1,-1,-1],
  'way_right' : [1,1,-1,1,1,-1],
  'leg_rod_length' : [0.156,0.12,0.045],
  }
  

  walk = walking(**Params)
  s = walk.setGoalVel(vel = [0,0,0])
  # print(s[0:2])
  # print(walk.next_leg)
  while True:
    while True:
      joint_angles,lf,rf,xp,n = walk.getNextPos()
      if(n == 0):
        break
    s = walk.setGoalVel(vel = [-0.05,0.03,0])
    # print(s[0:2])
    # print(walk.next_leg)
    
    while True:
      joint_angles,lf,rf,xp,n = walk.getNextPos()
      if(n == 0):
        break
    s = walk.setGoalVel(vel = [1,0,0])
    # print(s[0:2])
    # print(walk.next_leg)
  
