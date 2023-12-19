#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# original author: citbrains 
# preview control
# ver: 20230805

import numpy as np
import control
import control.matlab
import time
class preview_control():
  def __init__(self, dt, period, z, Q = 1.0e+8, H = 1.0):
    self.dt = dt
    self.period = period # optimize period
    G = 9.8
    A = np.matrix([
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0],
      [0.0, 0.0, 0.0]])
    B = np.matrix([[0.0], [0.0], [1.0]])
    C = np.matrix([[1.0, 0.0, -z/G]])
    D = 0
    sys = control.matlab.ss(A, B, C, D)
    
    # continuous to discrete
    sys_d = control.c2d(sys, dt)
    self.A_d, self.B_d, self.C_d, D_d = control.matlab.ssdata(sys_d)

    # modify car-table model
    Zero = np.matrix([[0.0], [0.0], [0.0]])
    Phai = np.block([[1.0, -self.C_d * self.A_d], [Zero, self.A_d]]) # A~
    G = np.block([[-self.C_d*self.B_d], [self.B_d]]) # b~
    GR = np.block([[1.0], [Zero]])
    
    # MPC optimize
    Qm = np.zeros((4,4))
    Qm[0][0] = Q
    P = control.dare(Phai, G, Qm, H)[0] # Qm -> Q; H -> R
    self.F = -np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P*Phai # Kx
    xi = (np.eye(4)-G*np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P)*Phai
    self.f = []
    self.xp, self.yp = np.matrix([[0.0],[0.0],[0.0]]), np.matrix([[0.0],[0.0],[0.0]])
    self.ux, self.uy = 0.0, 0.0
    for i in range(0,round(period/dt)):
      self.f += [-np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*np.linalg.matrix_power(xi.transpose(),i-1)*P*GR] # fi

  def set_param(self, t, current_x, current_y, foot_plan, pre_reset = False):

    #print('Caculate next step, step left ' + str(len(foot_plan) - 1))

    # set now state
    x, y = current_x.copy(), current_y.copy()
    if pre_reset == True:
      self.xp, self.yp = x.copy(), y.copy()
      self.ux, self.uy = 0.0, 0.0
    
    COG_X = []
    warn_flag = 0
    for i in range(round((foot_plan[1][0] - t) /self.dt)):
      
      # caculate Kp => now state feed back 
      px, py = self.C_d * x, self.C_d * y
      ex, ey = foot_plan[0][1] - px, foot_plan[0][2] - py
      X, Y = np.block([[ex], [x - self.xp]]), np.block([[ey], [y - self.yp]])
      self.xp, self.yp = x.copy(), y.copy()
      dux, duy = self.F * X, self.F * Y

      # caculate Kd => new target feed forward
      index = 1
      index_max = len(foot_plan) - 1
      for j in range(1, round(self.period / self.dt) - 1):
        # show opt error
        if index > index_max :
          if warn_flag == 0:
            print('warning: Partial motion frames in this step cannot obtain complete future ZMP feedforward ')
            warn_flag = 1
          break
        
        if round((i+j)+t/self.dt) >= round(foot_plan[index][0]/self.dt):
          dux += self.f[j] * (foot_plan[index][1]-foot_plan[index-1][1])
          duy += self.f[j] * (foot_plan[index][2]-foot_plan[index-1][2])
          index += 1
          
      self.ux, self.uy = self.ux + dux, self.uy + duy

      # caculate new state
      x, y = self.A_d * x + self.B_d * self.ux, self.A_d * y + self.B_d * self.uy
      COG_X += [np.block([x[0][0], y[0][0], px[0][0], py[0][0]])]
      # less CPU good for threading
      time.sleep(0.00001)

    return COG_X, x, y
  
if __name__ == '__main__':
  foot_width = 0.044
  period = 0.34
  pc = preview_control(0.01, period, 0.27)
  
  from foot_step_planner import *
  planner = foot_step_planner(0.06, 0.04, 0.1, period, foot_width)
  x, y = np.matrix([[0.0],[0.0],[0.0]]), np.matrix([[0.0],[0.0],[0.0]])

  import matplotlib.pyplot as plt
  
  tl = []
  pxl = []
  pyl = []
  pxtl =[]
  pytl =[]
  
  exit_flag = 0
  t_speed_change = 0

  foot_step = planner.calculate(0.04, 0.04, 0.0, 0.0, 0.0, 0.0, 'right','start')
  while len(foot_step) > 2:
    t = foot_step[0][0]
    cog, x, y = pc.set_param(t, x, y, foot_step)
    i_count = 0
    for i in cog:
      il = i.tolist()[0]
      pxl.append(il[2])
      pyl.append(il[3])
      pxtl.append(foot_step[0][1])
      pytl.append(foot_step[0][2])
      tl.append(t + i_count * 0.01 + t_speed_change)
      i_count += 1
  
    # next step
    del foot_step[0]

    if len(foot_step) < 10:
      if (exit_flag == 0) :
        current_x, current_y, current_leg = foot_step[0][1], foot_step[0][2], foot_step[0][4]
        foot_step = planner.calculate(0.03, 0.03, 0.0, current_x, current_y - foot_width, 0.0, current_leg, 'walking')
        exit_flag = 1
        t_speed_change = t
      else:
        pass

    
  fig, axs = plt.subplots(2, 1)
  axs[0].plot(tl, pxl, label = 'px')
  axs[0].plot(tl, pxtl, label = 'target px')
  axs[0].plot(tl, abs(np.array(pxtl) - np.array(pxl)) * 10, label = 'px track error x10')
  axs[0].legend()
  
  axs[1].plot(tl, pyl, label = 'py')
  axs[1].plot(tl, pytl, label= 'target py')
  axs[1].plot(tl, abs(np.array(pytl) - np.array(pyl)) * 10, label = 'py track error x10')
  axs[1].legend()
  plt.show()

  # import csv
  # with open('result.csv', mode='w', newline = '') as f:
  #   #f.write('')
  #   writer = csv.writer(f)
  #   writer.writerow(['x','y','px','py','stepx','stepy'])

  # while len(foot_step) > 0:
  #   t = foot_step[0][0]
  #   cog, x, y = pc.set_param(t, x, y, foot_step)
    
  #   # write csv
  #   with open('result.csv', mode='a', newline = '') as f:
  #     writer = csv.writer(f)
  #     for i in cog:
  #       il = i.tolist()[0]
  #       il.append(foot_step[0][1])
  #       il.append(foot_step[0][2])
  #       writer.writerow(il)
        
  #   # next step
  #   del foot_step[0] 
