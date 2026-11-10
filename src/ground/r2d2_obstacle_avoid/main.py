#!/usr/bin/env python3
import math
import time
import pybullet as p
import pybullet_data

# 连接 PyBullet 可视化仿真窗口（限制窗口尺寸，降低软件渲染负担）
client_id = p.connect(p.GUI, options="--width=960 --height=720")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

p.loadURDF("plane.urdf")

# 路上放两个箱子，错开摆，挡在前进方向上
def add_box(pos, half, color):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    p.createMultiBody(0, col, vis, pos)

add_box([0.4, 1.8, 0.5], [0.5, 0.5, 0.5], [0.75, 0.45, 0.3, 1])    # 偏右一个
add_box([-2.2, 4.5, 0.5], [0.5, 0.5, 0.5], [0.3, 0.5, 0.75, 1])     # 偏左一个

# r2d2 默认车头朝 -y，转 180 度让车头朝向 +y（障碍方向）
robot = p.loadURDF("r2d2.urdf", [0, -2, 0.2], p.getQuaternionFromEuler([0, 0, math.pi]))

# 按关节名称建立车轮映射（官方 r2d2.urdf：right_* 为右侧轮，left_* 为左侧轮）
right_wheels = []
left_wheels = []
for i in range(p.getNumJoints(robot)):
    joint_name = p.getJointInfo(robot, i)[1].decode("utf-8")
    if joint_name in ("right_front_wheel_joint", "right_back_wheel_joint"):
        right_wheels.append(i)
    elif joint_name in ("left_front_wheel_joint", "left_back_wheel_joint"):
        left_wheels.append(i)


def drive(v, w):
    for j in left_wheels:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=v + w, force=20)
    for j in right_wheels:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=v - w, force=20)


def yaw_of():
    _, orn = p.getBasePositionAndOrientation(robot)
    return p.getEulerFromQuaternion(orn)[2]


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# 前方射线探测，angle 是相对车头的偏角
# 实测 r2d2 默认车头朝 -y：车头方向 = (sin yaw, -cos yaw)
def front_dist(angle):
    pos, orn = p.getBasePositionAndOrientation(robot)
    yaw = p.getEulerFromQuaternion(orn)[2]
    dx = math.sin(yaw + angle)
    dy = -math.cos(yaw + angle)
    start = [pos[0] + 0.35 * dx, pos[1] + 0.35 * dy, 0.4]
    end = [pos[0] + 6 * dx, pos[1] + 6 * dy, 0.4]
    res = p.rayTest(start, end, robot)
    if res[0][0] < 0:
        return 999
    return 6 * res[0][2]


p.setRealTimeSimulation(1)

# 先转一下，确定 w 为正时 yaw 是增还是减
y0 = yaw_of()
drive(0, 2)
time.sleep(0.2)
drive(0, 0)
w_sign = 1 if norm_angle(yaw_of() - y0) > 0 else -1

home_yaw = yaw_of()


def rotate_to(target):
    while True:
        err = norm_angle(target - yaw_of())
        if abs(err) < 0.05:
            drive(0, 0)
            return
        drive(0, w_sign * 4 if err > 0 else -w_sign * 4)
        time.sleep(0.01)


try:
    print("出发，直行中")
    # 检测仿真连接状态：窗口被关闭时 isConnected 返回 False，循环自动退出
    while p.isConnected(client_id):
        pos, _ = p.getBasePositionAndOrientation(robot)
        if pos[1] > 5.5:
            drive(0, 0)
            print("到达终点，避障完成")
            break
        d = front_dist(0)
        if d < 0.8:
            drive(0, 0)
            time.sleep(0.1)
            print("检测到障碍，绕行")
            left_f = front_dist(0.6)     # 左前射线
            right_f = front_dist(-0.6)   # 右前射线
            if right_f < left_f:
                rotate_to(yaw_of() + math.pi / 2)   # 障碍偏右，往左绕
            else:
                rotate_to(yaw_of() - math.pi / 2)   # 障碍偏左，往右绕
            start = p.getBasePositionAndOrientation(robot)[0][:2]
            while True:
                cur = p.getBasePositionAndOrientation(robot)[0][:2]
                if math.hypot(cur[0] - start[0], cur[1] - start[1]) > 2.2:
                    break
                drive(8, 0)
                time.sleep(0.01)
            rotate_to(home_yaw)                     # 绕过去了，回正继续走
        else:
            err = norm_angle(home_yaw - yaw_of())
            if err > 0.03:
                drive(8, w_sign * 3)          # 需要 yaw 增大 → w 取 w_sign 方向
            elif err < -0.03:
                drive(8, -w_sign * 3)         # 需要 yaw 减小 → w 取反方向
            else:
                drive(8, 0)
        time.sleep(0.01)
except KeyboardInterrupt:
    # Ctrl+C 正常退出
    pass
finally:
    # 释放仿真连接
    p.disconnect(client_id)
    print("仿真已退出，连接已释放")
