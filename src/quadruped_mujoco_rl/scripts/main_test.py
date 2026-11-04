#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import mujoco
import os

def main():
    # 初始化 ROS 节点
    rospy.init_node('quadruped_rl_tester', anonymous=True)
    rospy.loginfo("MuJoCo RL Testing and Rendering Node Started...")

    # 自动获取 model 文件的绝对路径（假设 quadruped.xml 在包目录下的 models 文件夹里）
    # 路径逻辑：当前脚本的上一级目录 -> models 文件夹
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "..", "models", "quadruped.xml")

    # 检查文件是否存在
    if not os.path.exists(model_path):
        rospy.logerr("找不到模型文件: %s", model_path)
        rospy.loginfo("请下载一个 MuJoCo XML 模型并重命名为 quadruped.xml，放到 models 文件夹下。")
        return

    try:
        # 1. 加载 MuJoCo 模型和数据
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        rospy.loginfo("MuJoCo 模型加载成功！总关节数量: %d，自由度: %d", model.njnt, model.nv)
    except Exception as e:
        rospy.logerr("加载 MuJoCo 模型失败: %s", str(e))
        return

    # 2. 开始仿真循环
    rate = rospy.Rate(100) # 设置仿真频率为 100 Hz
    rospy.loginfo("开始执行 MuJoCo 仿真循环... (虚拟机中如遇闪退，请提前执行 export MUJOCO_GL=osmesa)")

    while not rospy.is_shutdown():
        # 3. 步进物理引擎
        mujoco.mj_step(model, data)

        # 4. 每秒打印一次状态信息 (输出机器人的基座位置和仿真时间)
        rospy.loginfo_throttle(1, "仿真时间: %.3f | 基座位置 (x, y, z): %s", 
                               data.time, data.qpos[:3])

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass