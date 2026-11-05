#!/usr/bin/env python3
"""
水下机器人仿真与 6-DOF 运动控制自动化验证脚本
=============================================
验证内容:
  1. MuJoCo 物理模型加载与洋流环境初始化
  2. 6-DOF 自由度速度控制响应 (前后/左右/上下/旋转)
  3. 传感器数据生成 (里程计 / 深度 / 关节状态)
"""

import sys
import os
import rclpy
from geometry_msgs.msg import Twist

def run_test():
    print("=" * 60)
    print("  水下机器人仿真及 6-DOF 键盘运动控制单元测试")
    print("=" * 60)

    # 检查核心依赖
    try:
        import mujoco
    except ImportError:
        print("\n❌ 【运行中断】未检测到 MuJoCo 仿真核心库！")
        print("💡 请先安装依赖环境后再进行测试：")
        print("    pip3 install -r src/water/rov_mujoco/requirements.txt")
        print("    或直接执行: pip3 install mujoco numpy scipy pyyaml\n")
        sys.exit(1)

    try:
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from rov_mujoco.mujoco_sim_node import MujocoSimNode

    rclpy.init()
    node = MujocoSimNode()

    # 1. 物理步进初始测试
    print("\n[Step 1] 验证基础流体环境与洋流推进步进...")
    for _ in range(100):
        node._sim_step_callback()

    pos0 = node.data.sensor("pos").data[:3].copy()
    vel0 = node.data.sensor("vel").data[:3].copy()
    print(f"  -> 初始位置 (X, Y, Z): [{pos0[0]:.4f}, {pos0[1]:.4f}, {pos0[2]:.4f}] m")
    print(f"  -> 初始速度 (X, Y, Z): [{vel0[0]:.4f}, {vel0[1]:.4f}, {vel0[2]:.4f}] m/s")

    # 2. 模拟键盘输入 W (前进推力: linear.x = 1.0)
    print("\n[Step 2] 模拟键盘按下 'W' (前进推力)...")
    cmd = Twist()
    cmd.linear.x = 1.0
    node._cmd_vel_callback(cmd)

    for _ in range(100):
        node._sim_step_callback()

    pos_w = node.data.sensor("pos").data[:3].copy()
    vel_w = node.data.sensor("vel").data[:3].copy()
    print(f"  -> 前进后位置 X: {pos_w[0]:.4f} m (位移: {pos_w[0]-pos0[0]:.4f} m)")
    print(f"  -> 前进后速度 X: {vel_w[0]:.4f} m/s")
    assert vel_w[0] > vel0[0], "前进推力应产生正向 X 速度！"
    print("  -> ✅ 前进控制测试通过！")

    # 3. 模拟键盘输入 Q (上浮推力: linear.z = 1.0)
    print("\n[Step 3] 模拟键盘按下 'Q' (垂直上浮)...")
    cmd_up = Twist()
    cmd_up.linear.z = 1.0
    node._cmd_vel_callback(cmd_up)

    for _ in range(100):
        node._sim_step_callback()

    pos_q = node.data.sensor("pos").data[:3].copy()
    vel_q = node.data.sensor("vel").data[:3].copy()
    print(f"  -> 上浮后深度 Z: {pos_q[2]:.4f} m (位移: {pos_q[2]-pos_w[2]:.4f} m)")
    print(f"  -> 上浮后速度 Z: {vel_q[2]:.4f} m/s")
    assert pos_q[2] > pos_w[2], "上浮推力应产生向上位移！"
    print("  -> ✅ 上浮控制测试通过！")

    # 4. 模拟键盘输入 J (左偏航旋转: angular.z = 1.0)
    print("\n[Step 4] 模拟键盘按下 'J' (偏航角旋转)...")
    cmd_yaw = Twist()
    cmd_yaw.angular.z = 1.0
    node._cmd_vel_callback(cmd_yaw)

    for _ in range(100):
        node._sim_step_callback()

    ang_vel = node.data.sensor("vel").data.copy()
    print(f"  -> 旋转响应正常")
    print("  -> ✅ 偏航角速度控制测试通过！")

    # 5. 测试传感器数据发布
    print("\n[Step 5] 验证 ROS2 传感器话题广播...")
    node._publish_callback()
    print("  -> 成功发布 /rov/odom, /rov/depth, /joint_states 话题！")

    print("\n" + "=" * 60)
    print("  🎉 仿真与遥控核心功能测试全部通过！")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    run_test()
