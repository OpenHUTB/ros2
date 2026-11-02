#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import mujoco
# import gymnasium as gym
# from stable_baselines3 import PPO

def main():
    rospy.init_node('quadruped_rl_trainer', anonymous=True)
    rospy.loginfo("MuJoCo RL Training Node Started...")
    
    # 1. 加载模型 (请确保你的 models 文件夹里有 quadruped.xml)
    try:
        model = mujoco.MjModel.from_xml_path("src/quadruped_mujoco_rl/models/quadruped.xml")
        data = mujoco.MjData(model)
        rospy.loginfo("MuJoCo Model loaded successfully. Total joints: %d", model.njnt)
    except Exception as e:
        rospy.logerr("Failed to load MuJoCo model: %s", str(e))
        return

    # 2. 模拟循环 (这里是训练或步态控制的逻辑)
    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        # 实际项目中，这里将执行 RL 策略推理并返回动作控制量
        # action = model.predict(observation)
        # data.ctrl[:] = action
        
        # 推进物理仿真
        mujoco.mj_step(model, data)
        
        # 打印一些状态信息
        rospy.loginfo_throttle(1, "Simulation time: %.2f", data.time)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass