#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# original author: citbrains 
# foot step planner
# ver: 20230805

class foot_step_planner():
  def __init__(self, max_stride_x, max_stride_y, max_stride_th, period, width, plan_step_num = 20):
  
    # max speed of step 
    self.max_stride_x = max_stride_x
    self.max_stride_y = max_stride_y
    self.max_stride_th = max_stride_th
    
    # step num
    self.bot_step_num = plan_step_num

    # record last command
    self.goal_vx_r = 0
    self.goal_vy_r = 0
    self.goal_th_r = 0
    
    # half T
    self.period = period
    # lenth between legs
    self.width = width

  def calculate(self, goal_vx, goal_vy, goal_vth, current_x, current_y, current_th, next_support_leg, status = 'walking'):
        
    # limit speed
    goal_vx = max(-self.max_stride_x, min(goal_vx, self.max_stride_x))
    goal_vy = max(-self.max_stride_y, min(goal_vy, self.max_stride_y))
    goal_vth = max(-self.max_stride_th, min(goal_vth, self.max_stride_th))  
    
    print('new command, speed after limit: ' + str([goal_vx, goal_vy, goal_vth])) 

    # strides in each direction
    stride_x  = goal_vx 
    stride_y  = goal_vy 
    stride_th = goal_vth

    # initial step => start
    time = 0.
    foot_step = []
    if status == 'start':
      print('start walk - zmp success ver 0.1')
      foot_step += [[0.0, current_x, current_y, current_th, 'both']]
    elif ((self.goal_vx_r == 0 and self.goal_vy_r == 0 and self.goal_th_r == 0) and (goal_vx != 0 or goal_vy != 0 or goal_vth != 0)):
      foot_step += [[0.0, current_x, current_y- self.width, current_th, 'both']]
    
    # walking step => normal
    next_x  = current_x
    next_y  = current_y
    next_th = current_th

           
    for i in range(self.bot_step_num):
      time += self.period
      
      if(goal_vx == 0 and goal_vy == 0 and goal_vth == 0):
        # v == 0
        if next_support_leg == 'right':
          foot_step += [[time, next_x, next_y, next_th, next_support_leg]]
          next_support_leg = 'both'
        elif next_support_leg == 'left' :
          foot_step += [[time, next_x, next_y, next_th, next_support_leg]]
          next_support_leg = 'both'
        else:
          foot_step += [[time, next_x, next_y, next_th, next_support_leg]]
          next_support_leg = 'both'
          
      else:
        # v != 0
        if next_support_leg == 'right':
          foot_step += [[time, next_x, next_y - self.width, next_th, next_support_leg]]
          next_support_leg = 'left'
        else:
          foot_step += [[time, next_x, next_y + self.width, next_th, 'left']]
          next_support_leg = 'right'         

      next_x  = current_x  + stride_x

      # gap step
      if((next_support_leg == 'right' and goal_vy < 0) or (next_support_leg == 'left' and goal_vy > 0)):
        next_y  = current_y  + stride_y
      else:
        next_y  = current_y

      next_th = current_th + stride_th
      current_x, current_y, current_th = next_x, next_y, next_th   
      
      self.goal_vx_r = goal_vx
      self.goal_vy_r = goal_vy
      self.goal_th_r = goal_vth
    return foot_step

if __name__ == '__main__':
  planner = foot_step_planner(0.06, 0.04, 0.1, 0.34, 0.044)
  foot_step = planner.calculate(1.0, 0.0, 0.5, 0.5, 0.0, 0.1, 'right')
  for i in foot_step:
    print(i)
