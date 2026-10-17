#!/usr/bin/env python3
import math
import time
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# 车库：后墙 + 左右墙
def add_box(pos, half, color):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    p.createMultiBody(0, col, vis, pos)

add_box([0, 2.5, 0.5], [1.1, 0.05, 0.5], [0.8, 0.8, 0.8, 1])      # 后墙
add_box([-1.1, 1.25, 0.5], [0.05, 1.25, 0.5], [0.8, 0.8, 0.8, 1])  # 左墙
add_box([1.1, 1.25, 0.5], [0.05, 1.25, 0.5], [0.8, 0.8, 0.8, 1])   # 右墙

# 小车从远处出发，先直行横穿库口前方，再弧线转弯倒车入库
robot = p.loadURDF("r2d2.urdf", [0.5, -2.8, 0.2], p.getQuaternionFromEuler([0, 0, math.pi / 2]))

left_wheels = [6, 7]   # 左轮
right_wheels = [2, 3]  # 右轮


def drive(v, w):
    for j in left_wheels:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=v + w, force=20)
    for j in right_wheels:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=v - w, force=20)


def yaw_of():
    _, orn = p.getBasePositionAndOrientation(robot)
    return p.getEulerFromQuaternion(orn)[2]


p.setRealTimeSimulation(1)

# 阶段一：直行，横穿库口前方
print("直行到库口侧方")
while True:
    x, _, _ = p.getBasePositionAndOrientation(robot)[0]
    if x >= 1.8:
        drive(0, 0)
        break
    drive(8, 0)
    time.sleep(0.01)

# 阶段二：弧线转弯倒车入库
print("弧线转弯倒车入库")
done = False
while True:
    pos, _ = p.getBasePositionAndOrientation(robot)
    x, y = pos[0], pos[1]
    if not done and y > 0.6:
        drive(0, 0)
        print("入库成功 位置 x=%.2f y=%.2f" % (pos[0], pos[1]))
        done = True
        continue
    if done:
        drive(0, 0)
        time.sleep(0.01)
        continue
    # 车尾对准库中心，边倒边转（大弧线甩尾）
    target = math.atan2(1.5 - y, -x)
    err = target - (math.pi / 2 + yaw_of())
    while err > math.pi:
        err -= 2 * math.pi
    while err < -math.pi:
        err += 2 * math.pi
    if y > -0.5 and abs(x) > 0.68:
        drive(-8, 0)   # 靠近库口太偏就直倒拉回
    elif abs(err) > 0.15:
        drive(-8, -6 if err < 0 else 6)
    else:
        drive(-8, 0)
    time.sleep(0.01)
