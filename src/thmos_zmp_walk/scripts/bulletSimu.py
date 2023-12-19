import pybullet as p
import time
import math
import pybullet_data
import numpy as np
if __name__=="__main__":
    # 连接物理引擎
    physicsCilent = p.connect(p.GUI)

    # 添加资源路径
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 设置环境重力加速度
    p.setGravity(0, 0, -9.8)

    # 加载URDF模型，此处是加载蓝白相间的陆地
    planeId = p.loadURDF("plane.urdf")
    #loadURDF是绝对路径
    # 加载机器人，并设置加载的机器人的位姿
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    
    robotId = p.loadURDF("D:/THMOS/git workspace/walk_feedback/src/walk_feedback/scripts/thmos_urdf/urdf/thmos_urdf.urdf", startPos, startOrientation)

    # 按照位置和朝向重置机器人的位姿，由于我们之前已经初始化了机器人，所以此处加不加这句话没什么影响
    p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)

    #是否打印机器人四维坐标
    getrobotpos=0
    current_state=0
    state_durations=[1,3,3]
    state_t=0
    
    step_time=1/240#步进时间
    #循环迭代，每次间隔1/240s
    
    while p.isConnected():
        p.stepSimulation()
        time.sleep(step_time)
        state_t+=step_time
        # 获取位置与方向四元数
        if getrobotpos:
            cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
            print("-" * 20)
            print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
            print("-" * 20)
