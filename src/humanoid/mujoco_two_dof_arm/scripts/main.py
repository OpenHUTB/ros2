#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 两自由度机械臂简单控制
实现：关节1和关节2做正弦摆动，展示简单的位置控制
"""

import mujoco
import mujoco.viewer
import numpy as np
import sys
import rospy

def main():
    # 优先读ROS私有参数里的模型路径，兼容roslaunch启动；没有参数就用默认路径
    rospy.init_node("mujoco_arm", anonymous=True, disable_signals=True)
    model_path = rospy.get_param("~model_path", "../models/two_dof_arm.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("=" * 50)
    print("MuJoCo 两自由度机械臂简单控制")
    print("=" * 50)
    print("控制方式：两个关节做正弦摆动")
    print("  关节1（肩关节）：±30度，周期2秒")
    print("  关节2（肘关节）：±45度，周期1.5秒")
    print("=" * 50)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        step = 0
        while viewer.is_running():
            t = data.time
            
            # 正弦位置控制：简单的周期性运动
            target_joint1 = 0.5 * np.sin(t * np.pi)    # ±0.5 rad ≈ ±30度
            target_joint2 = 0.8 * np.sin(t * 1.3 * np.pi)  # ±0.8 rad ≈ ±45度
            
            # 写入位置控制目标
            data.ctrl[0] = target_joint1  # 关节1
            data.ctrl[1] = target_joint2   # 关节2
            
            # 步进仿真
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 每0.5秒打印一次状态
            step += 1
            if step % 100 == 0:
                j1 = data.sensordata[0]
                j2 = data.sensordata[1]
                ee_x = data.sensordata[2]
                ee_y = data.sensordata[3]
                ee_z = data.sensordata[4]
                print(f"\r关节1: {j1:.2f}rad | 关节2: {j2:.2f}rad | 末端位置: ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f})", end="")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出控制")
