#!/usr/bin/env python3

import sys
#print(sys.path)
import rospy
from geometry_msgs.msg import Twist
# webots version: from bitbots_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
import numpy as np
class motor_offset:
    def __init__(self, base_ns = ''):
        param,param_leg=self.read_txt()
        self.Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'full_leg_length' : param[6],
            'optimize_period' : param[7],
            'walking_period' : param[8],
            'both_foot_support_time' : param[9],
            'dt' : param[10],
            'max_vx' : param[11],
            'max_vy': param[12],
            'max_vth' : param[13],
            'plan_step_num' :int(param[14]),
            'k_offset':param[15],#ex_com_y_offset k
            'b_offset':param[16],#ex_com_y_offset b
            'way_left' : param_leg[0],
            'way_right' : param_leg[1],
            'motor_offset_left' :param_leg[2],
            'motor_offset_right' : param_leg[3],
            'leg_rod_length' : [0.156,0.12,0.045]
            }
        
        
        self.joint_goal_msg = JointState()
        self.joint_goal_msg.name = ["R_leg_1", "R_leg_2", "R_leg_3", "R_leg_4", "R_leg_5", "R_leg_6",
                                           "L_leg_1", "L_leg_2", "L_leg_3", "L_leg_4", "L_leg_5", "L_leg_6"]  
                                           
        self.joint_goal_msg.velocity = [3.14] * 12
        self.joint_angles_raw = self.Params['motor_offset_right'] +  self.Params['motor_offset_left']

        self.next_walk_goal = [0.0, 0.0, 0.0] 
        self.old_walk_goal = [0.0, 0.0, 0.0]
        self.speed_change_flag = 1

        self.step_pic_remain = 0
        self.rate = rospy.Rate(100)  #100    

        self.joint_goal_publisher = rospy.Publisher(base_ns + '/walking_motor_goals', JointState, queue_size=1)
    def read_txt(self):
        # sys.path.append(sys.path[0] + '/param.txt')
        # param_path=sys.path[-1]
        param_path='/home/nvidia/params_ws/src/walk_params/zmp_walk/param.txt'
        param=np.genfromtxt(fname=param_path,dtype=float,
delimiter=",",comments="#",max_rows=34,invalid_raise=False)
        print(param)
        param_leg=np.genfromtxt(fname=param_path,dtype=float,
delimiter=",",comments="#",skip_header=35, max_rows=42,invalid_raise=False)
        print(param_leg)
        return param,param_leg
    def pub_joint_goal(self):
        self.joint_goal_msg.position = self.joint_angles_raw
        self.joint_goal_publisher.publish(self.joint_goal_msg)  
    
    def walk(self):   
        self.rate.sleep()
        self.pub_joint_goal()
        
    
if __name__ == "__main__":
    rospy.init_node('thmos_motor_offset', anonymous=True) 
    walk = motor_offset()
    for i in range(10):
      walk.walk()
