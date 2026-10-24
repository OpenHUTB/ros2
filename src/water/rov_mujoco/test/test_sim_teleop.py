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
import time
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

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

    # 4. 模拟键盘输入 A (左横移: linear.y = 1.0)
    print("\n[Step 4] 模拟键盘按下 'A' (左横移)...")
    cmd_strafe = Twist()
    cmd_strafe.linear.y = 1.0
    node._cmd_vel_callback(cmd_strafe)

    for _ in range(100):
        node._sim_step_callback()

    pos_a = node.data.sensor("pos").data[:3].copy()
    vel_a = node.data.sensor("vel").data[:3].copy()
    print(f"  -> 横移后位置 Y: {pos_a[1]:.4f} m (位移: {pos_a[1]-pos_q[1]:.4f} m)")
    print(f"  -> 横移后速度 Y: {vel_a[1]:.4f} m/s")
    assert pos_a[1] - pos_q[1] > 0.005, "左横移应产生正向 Y 位移！"
    assert vel_a[1] > 0.0, "左横移应产生正向 Y 速度！"
    print("  -> ✅ 横移控制测试通过！")

    # 5. 模拟键盘输入 J (左偏航旋转: angular.z = 1.0)
    print("\n[Step 5] 模拟键盘按下 'J' (偏航角旋转)...")
    cmd_yaw = Twist()
    cmd_yaw.angular.z = 1.0
    node._cmd_vel_callback(cmd_yaw)

    for _ in range(100):
        node._sim_step_callback()

    gyro_z = float(node.data.sensor("gyro").data[2])
    print(f"  -> 偏航角速度 (gyro.z): {gyro_z:+.4f} rad/s")
    assert gyro_z > 0.02, f"左偏航应产生正向角速度，实际 {gyro_z:.4f} rad/s！"

    cmd_yaw.angular.z = -1.0
    node._cmd_vel_callback(cmd_yaw)
    for _ in range(100):
        node._sim_step_callback()

    gyro_z_rev = float(node.data.sensor("gyro").data[2])
    print(f"  -> 反向偏航角速度 (gyro.z): {gyro_z_rev:+.4f} rad/s")
    assert gyro_z_rev < -0.02, f"右偏航应产生负向角速度，实际 {gyro_z_rev:.4f} rad/s！"
    print("  -> ✅ 偏航角速度控制测试通过 (含反向)！")

    # 6. 键盘映射与急停/速度比例（不依赖物理，直接验证节点逻辑）
    print("\n[Step 6] 验证键盘键位映射、急停与速度比例...")
    try:
        from rov_mujoco.keyboard_teleop_node import KeyboardTeleopNode, KEY_BINDINGS
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from rov_mujoco.keyboard_teleop_node import KeyboardTeleopNode, KEY_BINDINGS

    expected = {
        'j': ('angular', 'z', 1.0), 'l': ('angular', 'z', -1.0),
        'i': ('angular', 'y', 1.0), 'k': ('angular', 'y', -1.0),
        'u': ('angular', 'x', -1.0), 'o': ('angular', 'x', 1.0),
        'w': ('linear', 'x', 1.0), 's': ('linear', 'x', -1.0),
        'a': ('linear', 'y', 1.0), 'd': ('linear', 'y', -1.0),
        'q': ('linear', 'z', 1.0), 'e': ('linear', 'z', -1.0),
    }
    for key, binding in expected.items():
        assert KEY_BINDINGS.get(key) == binding, \
            f"键位 {key!r} 映射错误: {KEY_BINDINGS.get(key)!r} != {binding!r}"
    print(f"  -> 12 个字母键位映射全部正确")

    kb = KeyboardTeleopNode()
    kb._get_key = lambda timeout=0.05: 'j'
    kb._publish_callback()
    assert kb.twist.angular.z > 0.0, "按下 J 应产生正偏航指令！"

    kb._get_key = lambda timeout=0.05: 'o'
    kb._publish_callback()
    assert kb.twist.angular.x > 0.0, "按下 O 应产生正翻滚指令！"

    kb.twist.linear.x = 1.0
    kb.twist.angular.z = 1.0
    kb._get_key = lambda timeout=0.05: ' '
    kb._publish_callback()
    assert (abs(kb.twist.linear.x) + abs(kb.twist.linear.y) + abs(kb.twist.linear.z)
            + abs(kb.twist.angular.x) + abs(kb.twist.angular.y) + abs(kb.twist.angular.z)) == 0.0, \
        "空格急停后所有速度指令必须归零！"

    old_scale = kb.linear_scale
    kb._get_key = lambda timeout=0.05: '+'
    kb._publish_callback()
    assert kb.linear_scale > old_scale, "按 '+' 应增大速度比例！"
    kb._get_key = lambda timeout=0.05: '-'
    kb._publish_callback()
    assert abs(kb.linear_scale - old_scale) < 1e-9, "按 '-' 应减小回原速度比例！"

    kb.destroy_node()
    print("  -> ✅ 键位映射、急停、速度比例测试通过！")

    # 7. 测试传感器数据发布（真实订阅并计数）
    print("\n[Step 7] 验证 ROS2 传感器话题广播...")
    counts = {'odom': 0, 'depth': 0, 'joint': 0}
    node.create_subscription(Odometry, '/rov/odom',
                             lambda msg: counts.__setitem__('odom', counts['odom'] + 1), 10)
    node.create_subscription(Float64, '/rov/depth',
                             lambda msg: counts.__setitem__('depth', counts['depth'] + 1), 10)
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: counts.__setitem__('joint', counts['joint'] + 1), 10)

    time.sleep(0.2)  # 等待 DDS 发现
    for _ in range(30):
        node._publish_callback()
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"  -> 收到 odom={counts['odom']}  depth={counts['depth']}  joint_states={counts['joint']}")
    assert counts['odom'] > 0, "未收到 /rov/odom 消息！"
    assert counts['depth'] > 0, "未收到 /rov/depth 消息！"
    assert counts['joint'] > 0, "未收到 /rov/joint_states 消息！"
    print("  -> ✅ 传感器话题发布测试通过！")

    print("\n" + "=" * 60)
    print("  🎉 仿真与遥控核心功能测试全部通过 (7/7)！")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    run_test()
