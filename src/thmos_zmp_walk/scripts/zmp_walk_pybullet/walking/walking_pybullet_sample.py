

import pybullet as p
import pybullet_data
import numpy as np
import sys
import os
sys.path.append('./walking')
from walking_feedback import *
from random import random 
from time import sleep
import threading 
import time
from zmp_imu import *

def read_txt():
        # sys.path.append(sys.path[0] + '/param.txt')
        # param_path=sys.path[-1]
        param_path="param.txt"	
        param=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",max_rows=34,invalid_raise=False)
        print(param)
        param_leg=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",skip_header=35, max_rows=46,invalid_raise=False)
        print(param_leg)
        return param,param_leg

if __name__ == '__main__':
  TIME_STEP = 0.001
  physicsClient = p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.8)
  p.setTimeStep(TIME_STEP)

  planeId = p.loadURDF("plane.urdf", [0, 0, 0])
  urdfpath =os.path.dirname(os.path.abspath(__file__)) + "/../urdf/thmos_urdf.urdf"
  print(urdfpath)
  startPos = [0, 0, 0.37]
  startOrientation = p.getQuaternionFromEuler([0, 0, 0])
  RobotId = p.loadURDF(urdfpath, startPos, startOrientation, useFixedBase = False)  #set False to detach the robot


  index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

  joint_angles = []
  for id in range(p.getNumJoints(RobotId)):
    if p.getJointInfo(RobotId, id)[3] > -1:
      joint_angles += [0,]

  index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

  # control box ----
  param,param_leg=read_txt()
  Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'trunk_height' : param[6],
            'optimize_period' : param[7],
            'walking_period' : param[8],
            'both_foot_support_time' : param[9],
            'dt' : param[10],
            'max_vx' : param[11],
            'max_vy': param[12],
            'max_vth' : param[13],
            'plan_step_num' : int(param[14]),
            'k_offset':param[15],#ex_com_y_offset k
            'b_offset':param[16],#ex_com_y_offset b
            'way_left' : param_leg[0],
            'way_right' : param_leg[1],
            'leg_rod_length' : [0.156,0.12,0.045],
            'motor_offset_left' :param_leg[2],
            'motor_offset_right' :param_leg[3],
            'full_leg_length' :0.31,
            'feedback_coef' :[0,0,0.006], #(yaw->th, pitch->x, roll->y : 0.006)
            'feedback_rotation_coef' :1.13137, #0.8*sqrt(2)

            }
  print("---------\nParams=\n", Params)
  walk = walking(RobotId, **Params)
  j = 0
  n = 0
  k = 0
  
  event_v = threading.Event()
  event_w = threading.Event()  
  
  # add pybullet
  robo = THMOSGym(RobotId)
  head_arm = [0,0, 0.52,1.22,1.5, -0.52,-1.22,-1.5]
  leg_right = [0,0,0, 0,0,0]
  leg_left = [0,0,0, 0,0,0]
  robo.RoboStep(head_arm + leg_right + leg_left,maxForce=12.5)
  robo.getForceInfo()
  # end add
  # vx,vy=robo.getLocalV()
  # print("Vx:{vx},Vy:{vy}")
  def set_vel_loop():
    while True:
      event_v.wait()
      walk.setGoalVel([ 0.05, 0, 0])
      event_w.set()
      event_v.clear()
        
  thread_get_vel = threading.Thread(target = set_vel_loop)
  thread_get_vel.start()
  event_w.set()
  
  while p.isConnected():
    j += 1
    if j >= 10:
      if n == 0:
        event_w.wait()
        walk.updateWalkGoal()
        event_v.set()
        event_w.clear()
      
      joint_angles,lf,rf,xp,n = walk.getNextPos()
      j = 0
    
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        if 'leg' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):
          p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force=8.5) # R_leg_1 to L_leg_6: 15-26
    sleep(0.001)
    p.stepSimulation()
