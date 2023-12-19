#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        THMOSPybulletPacket.py
# Creator:          JinYin Zhou
# Version:          0.1
# Created:          2023/2/19
# Description:      simulation program support packet
# Function List:        class THMOSGym:
#                       RoboStep() -- simulation for one step
#                       getEnvInfo() -- all kinematic parameters are obtained
#                       buildRobo() -- initialization
# History:
#   <author>      <version>       <time>          <description>
#   Jinyin Zhou     0.1           2023/2/19       create
# -----------------------------------------------------

import pybullet as p
import time
import sys
import math
import pybullet_data

class THMOSGym:
    """It's a Gym for THMOS"""
    def __init__(self, RobotId):
        """initialize class"""
        # build robot
        self.quadruped,self.jointIds,self.jointNum = self.buildRobo(RobotId)
        print("robot id : ", self.quadruped)
        
        # joint command
        self.move = [0] * self.jointNum
        self.ax = 0
        self.ay = 0
        
        # realtime simulation : 1 = true
        p.setRealTimeSimulation(1)

    def buildRobo(self,RobotId):
        """ Set and bulid the virtual world """
        
        urdfFlags = p.URDF_USE_SELF_COLLISION #启用自碰撞
        quadruped=RobotId
        jointIds = []
  
        # get joints states
        jointNum = 0
        for j in range (p.getNumJoints(quadruped)):
            # set damping 去掉节点滑动和转动阻尼
            p.changeDynamics(quadruped,j,linearDamping = 0, angularDamping = 0)
            # get motor info
            info = p.getJointInfo(quadruped,j)
            # print("info_print:" ,info)
            jointName = info[1]
            jointType = info[2]
            # print(jointName, "joint id:", j)
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                jointIds.append(j)
                jointNum = jointNum + 1

        # enable sole force sensor
        p.enableJointForceTorqueSensor(quadruped,20,True)
        p.enableJointForceTorqueSensor(quadruped,27,True)

        print("THMOS is ready!")
        return quadruped, jointIds, jointNum

    def RoboStep(self,ActorOut,maxForce=12.5):
        """set motor angle by absolute angle [rad]
        Args:
            ActorOut: absolute angle [rad]
            maxForce: motor max torque [N*m]
        """

        # make control command
        for j in range (self.jointNum):
            self.move[j] = float(ActorOut[j])
            p.setJointMotorControl2(self.quadruped, self.jointIds[j], p.POSITION_CONTROL, self.move[j], force = maxForce)
        
        # sends control command
        p.stepSimulation()

        # Control interval
        # time.sleep(self.TimeStep)


    def getEnvInfo(self):
        """
         Returns:
            envInfo:the dynamics information of robot upper body[Vx, Vy, Vz, wx, wy, wz, x, y, z, roll, pitch, yaw]
        """
        linearVel, angularVel = p.getBaseVelocity(self.quadruped)
        bodyPos, bodyOrn = p.getBasePositionAndOrientation(self.quadruped)
        bodyEuler = p.getEulerFromQuaternion(bodyOrn)
        envInfo = list(linearVel+angularVel+bodyPos+bodyEuler)
        return envInfo
 
   
    def getForceInfo(self):
        """
         Returns:
            FroceInfo : the react force of ground
        """
        r_sole_force = p.getJointState(self.quadruped, 20)[2]
        l_sole_force = p.getJointState(self.quadruped, 27)[2]
        return r_sole_force, l_sole_force

    def getSolePos(self):
        """
         Returns:
            sole pos Info : the pos of sole
        """
        r_sole_pos = p.getLinkState(self.quadruped, 20)[0]
        l_sole_pos = p.getLinkState(self.quadruped, 27)[0]
        return r_sole_pos, l_sole_pos

    def paintZmpInfo(self,dt = 0.2):
        """
         paint zmp
        """
        r_f, l_f = self.getForceInfo()
                
        # right zmp
        if(r_f[2] < - 2):
            pxr = - r_f[4] / r_f[2]
            pyr = r_f[5] / r_f[2]
            r_t = 0.5
        else:
            pxr = 0
            pyr = 0
            r_t = 0
                    
        # left zmp
        if(l_f[2] < - 2):
            pxl = - l_f[4] / l_f[2]
            pyl = l_f[5] / l_f[2]
            l_t = 0.5
        else:
            pxl = 0
            pyl = 0
            l_t = 0
                    
        # paint zmp point
        r_sole_pos, l_sole_pos = self.getSolePos()

        pxr = r_sole_pos[0] + pxr
        pyr = r_sole_pos[1] + pyr
        pxl = l_sole_pos[0] + pxl
        pyl = l_sole_pos[1] + pyl

        if(r_f[2] + l_f[2] < - 2):
            px = (pxr * r_f[2] + pxl * l_f[2]) / (r_f[2] + l_f[2])
            py = (pyr * r_f[2] + pyl * l_f[2]) / (r_f[2] + l_f[2])
            a_t = 0.5
        else:
            px = (pxr + pxl) / 2
            py = (pyr + pyl) / 2
            a_t = 0.5            
                
        p.addUserDebugLine([px,py,0], [px,py,a_t], lineColorRGB=[1, 0, 0], lifeTime = dt, lineWidth = 3)

        return [px - (pxr + pxl) / 2, py - (pyr + pyl) / 2]
    
    def rot_yaw(self,x,y,yaw):
        """
        rotate 2D pos
        """
        xr = x * math.cos(yaw) - y * math.sin(yaw)
        yr = x * math.sin(yaw) + y * math.cos(yaw)
        return xr , yr

    def paintSupportArea(self,dt = 0.2):
        """
         paint support graph
        """
        dx = 0.06
        dy = 0.04

        r_sole_pos, l_sole_pos = self.getSolePos()
        envinfo = self.getEnvInfo()
        yaw = envinfo[11]

        dx,dy = self.rot_yaw(dx, dy, yaw)
        # paint sole
        xr1 = r_sole_pos[0] + dx
        yr1 = r_sole_pos[1] + dy
        xr2 = r_sole_pos[0] - dx
        yr2 = r_sole_pos[1] - dy

        xl1 = l_sole_pos[0] + dx
        yl1 = l_sole_pos[1] + dy
        xl2 = l_sole_pos[0] - dx
        yl2 = l_sole_pos[1] - dy   
         
        p.addUserDebugLine([xr1,yr1,0.01], [xr2,yr2,0.01], lineColorRGB=[0, 0, 1], lifeTime = dt, lineWidth = 3)
        p.addUserDebugLine([xr2,yr2,0.01], [xl2,yl2,0.01], lineColorRGB=[0, 0, 1], lifeTime = dt, lineWidth = 3)
        p.addUserDebugLine([xl2,yl2,0.01], [xl1,yl1,0.01], lineColorRGB=[0, 0, 1], lifeTime = dt, lineWidth = 3)
        p.addUserDebugLine([xl1,yl1,0.01], [xr1,yr1,0.01], lineColorRGB=[0, 0, 1], lifeTime = dt, lineWidth = 3)

        return 0

    def getLocalImuZmp(self, dt = 0.2, a = 0.02, b = 0.1, K = 5):
        """
         paint zmp caculate by imu
         get local zmp point
        """
        envinfo = self.getEnvInfo()
        yaw_z = envinfo[11]
        # local acc
        ax_l, ay_l = self.rot_yaw(envinfo[0], envinfo[1], -yaw_z)
        self.ax = b * ax_l + (1 - b) * self.ax * 0.05
        self.ay = a * ay_l + (1 - a) * self.ay  #what's meaning?
        # world zmp in local frame
        zmp_x_w, zmp_y_w = self.rot_yaw(K * self.ax,K * self.ay, yaw_z) 
        # paint zmp
        r_sole_pos, l_sole_pos = self.getSolePos()
        zmp_x_p = zmp_x_w + (r_sole_pos[0] + l_sole_pos[0]) / 2
        zmp_y_p = zmp_y_w + (r_sole_pos[1] + l_sole_pos[1]) / 2
        p.addUserDebugLine([zmp_x_p,zmp_y_p,0], [zmp_x_p,zmp_y_p,0.5], lineColorRGB=[0, 1, 1], lifeTime = dt, lineWidth = 5)
        return zmp_x_w,zmp_y_w
    
    def getLocalV(self, Kvx, Kvy):
        """
         get local vx, vy
        """
        envinfo = self.getEnvInfo()
        vx, vy = self.rot_yaw(envinfo[0], envinfo[1], -envinfo[11])
        return [Kvx * vx,Kvy * vy]

if __name__ == '__main__':
    timestep=0.01
    p.connect(p.GUI)
    robo = THMOSGym(0.01,True)
    # ground building
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    # THMOS - robot building
    path = sys.argv[0] + "/../urdf/thmos_urdf.urdf"
    quadruped = p.loadURDF(path, [0, 0, 0.36], [0, 0, 0, 1], useFixedBase=False)
    head_arm = [0,0, 0,0,0, 0,0,0]
    leg_right = [0,0,0, 0,0,0]
    leg_left = [0,0,0, 0,0,0]
    robo.RoboStep(head_arm + leg_right + leg_left,maxForce=12.5)
    robo.getForceInfo()
    # while 1:
    #     robo.getLocalImuZmp()
    time.sleep(2000)