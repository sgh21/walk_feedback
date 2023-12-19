#!/usr/bin/env python

import sys
sys.path.append(sys.path[0] + '/walking')
#print(sys.path)
import rospy
import threading
from geometry_msgs.msg import Twist
# webots version: from bitbots_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from walking import walking
import numpy as np

class zmp_walker:
    def __init__(self, base_ns = ''):
        param,param_leg=self.read_txt()
        self.Params = {              
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
            'full_leg_length' :param_leg[5],
            'feedback_coef' :[1.0,1.0,1.0], #(yaw->th, pitch->x, roll->y)
            'feedback_rotation_coef' :1.13137, #0.8*sqrt(2)

            }
        
        self.walk_gen = walking(** self.Params)
        
        self.joint_goal_msg = JointState()
        self.joint_goal_msg.name = ["R_leg_1", "R_leg_2", "R_leg_3", "R_leg_4", "R_leg_5", "R_leg_6",
                                           "L_leg_1", "L_leg_2", "L_leg_3", "L_leg_4", "L_leg_5", "L_leg_6"]  
                                           
        self.joint_goal_msg.velocity = [3.14] * 12
        self.joint_angles_raw = [0.0] * 12

        self.next_walk_goal = [0.0, 0.0, 0.0] 
        self.old_walk_goal = [0.0, 0.0, 0.0]
        self.speed_change_flag = 1

        self.step_pic_remain = 0
        self.rate = rospy.Rate(100)  #100    

        self.walk_goal_subscriber = rospy.Subscriber(base_ns + '/cmd_vel', Twist, self.walk_goal_callback, queue_size=1, tcp_nodelay=True)
        self.joint_goal_publisher = rospy.Publisher(base_ns + '/walking_motor_goals', JointState, queue_size=1)

        self.event_v = threading.Event()
        self.event_w = threading.Event()
        self.thread_get_vel = threading.Thread(target = self.set_vel_loop)
        self.thread_get_vel.start()
        self.event_w.set()
    def read_txt(self):
        # sys.path.append(sys.path[0] + '/param.txt')
        # param_path=sys.path[-1]
        param_path='/home/nvidia/params_ws/src/walk_params/zmp_walk/param.txt'		
        param=np.genfromtxt(fname=param_path,dtype=float,
delimiter=",",comments="#",max_rows=34,invalid_raise=False)
        print(param)
        param_leg=np.genfromtxt(fname=param_path,
dtype=float,delimiter=",",comments="#",skip_header=35, max_rows=46,invalid_raise=False)
        print(param_leg)
        return param,param_leg
    def set_vel_loop(self):
      while True:
        self.event_v.wait()
        self.walk_gen.setGoalVel(self.next_walk_goal)
        self.event_w.set()
        self.event_v.clear()

    def walk_goal_callback(self, walk_goal_msg):
        self.next_walk_goal = [walk_goal_msg.linear.x, walk_goal_msg.linear.y, walk_goal_msg.angular.z]
        if (self.next_walk_goal[0] == self.old_walk_goal[0] and self.next_walk_goal[1] == self.old_walk_goal[1] and self.next_walk_goal[2] == self.old_walk_goal[2]):
            self.speed_change_flag = 0
        else:
            self.speed_change_flag = 1
        self.old_walk_goal = [walk_goal_msg.linear.x, walk_goal_msg.linear.y, walk_goal_msg.angular.z]
        #k_offset=-0.15
        #b_offset=-0.006#add these to param
        self.walk_gen.com_y_offset=self.Params['k_offset']*self.next_walk_goal[1]+self.Params['b_offset']
        
    def pub_joint_goal(self):
        self.joint_goal_msg.position = self.joint_angles_raw
        self.joint_goal_publisher.publish(self.joint_goal_msg)  
    
    def walk(self):
        if self.step_pic_remain == 0:
            if(self.speed_change_flag == 1):
                self.event_w.wait()
                self.walk_gen.updateWalkGoal()
                self.speed_change_flag = 0
                self.event_v.set()
                self.event_w.clear()
            else:
                self.event_w.wait()
                self.walk_gen.updateWalkGoal()
                self.event_v.set()
                self.event_w.clear()
        self.joint_angles_raw, _, _, _, self.step_pic_remain = self.walk_gen.getNextPos()
        self.rate.sleep()
        self.pub_joint_goal()
        
    
if __name__ == "__main__":
    rospy.init_node('thmos_zmp_walk', anonymous=True) 
    walk = zmp_walker()
          
    while not rospy.is_shutdown():
      walk.walk()
