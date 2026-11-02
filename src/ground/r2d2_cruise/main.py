#!/usr/bin/env python3
import pybullet as p
import pybullet_data
import time

# 连接 PyBullet 可视化仿真窗口（限制窗口尺寸，降低软件渲染负担）
client_id = p.connect(p.GUI, options="--width=960 --height=720")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf", [0, 0, 0.2])
p.setRealTimeSimulation(1)

# 按关节名称建立车轮映射（官方 r2d2.urdf：right_* 为右侧轮，left_* 为左侧轮）
right_wheels = []
left_wheels = []
for i in range(p.getNumJoints(robotId)):
    joint_name = p.getJointInfo(robotId, i)[1].decode("utf-8")
    if joint_name in ("right_front_wheel_joint", "right_back_wheel_joint"):
        right_wheels.append(i)
    elif joint_name in ("left_front_wheel_joint", "left_back_wheel_joint"):
        left_wheels.append(i)

# 固定轮速差速圆周运动示例：
# 左轮速度高、右轮速度低 -> 左右轮产生速度差，小车沿圆弧轨迹持续圆周巡航
# （与演示动图录制时的行驶方向保持一致）
left_speed = 10.0
right_speed = 2.0
force = 20


def set_wheel_speed(joints, speed):
    """设置一组车轮的目标速度"""
    for j in joints:
        p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL,
                                targetVelocity=speed, force=force)


try:
    # 检测仿真连接状态：窗口被关闭时 isConnected 返回 False，循环自动退出
    while p.isConnected(client_id):
        set_wheel_speed(left_wheels, left_speed)
        set_wheel_speed(right_wheels, right_speed)
        time.sleep(0.01)
except KeyboardInterrupt:
    # Ctrl+C 正常退出
    pass
finally:
    # 释放仿真连接
    p.disconnect(client_id)
    print("仿真已退出，连接已释放")
