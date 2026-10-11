import pybullet as p
import pybullet_data
import time
import math

# 连接仿真窗口
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载地面
planeId = p.loadURDF("plane.urdf")

# 创建小车底盘
carStartPos = [0, 0, 0.1]
carStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
car = p.loadURDF("r2d2.urdf", carStartPos, carStartOrientation)

# 放置障碍物
obs1 = p.loadURDF("cube.urdf", [2, 0, 0.5], globalScaling=1.0)
obs2 = p.loadURDF("cube.urdf", [0, 2, 0.5], globalScaling=1.0)
obs3 = p.loadURDF("cube.urdf", [-2, 0, 0.5], globalScaling=1.0)

# 车轮关节编号
wheel_joints = [2, 3, 6, 7]
max_vel = 8.0
force = 10

# 四路激光：前、右、后、左
laser_height = 0.2
laser_range = 3.0
laser_dirs = [
    [1, 0, 0],   # 前
    [0, 1, 0],   # 右
    [-1, 0, 0],  # 后
    [0, -1, 0]   # 左
]

def get_laser_distances():
    pos, orn = p.getBasePositionAndOrientation(car)
    rot = p.getMatrixFromQuaternion(orn)
    rot = [rot[0:3], rot[3:6], rot[6:9]]
    dists = []
    for d in laser_dirs:
        # 转换到世界坐标系
        dx = rot[0][0]*d[0] + rot[0][1]*d[1] + rot[0][2]*d[2]
        dy = rot[1][0]*d[0] + rot[1][1]*d[1] + rot[1][2]*d[2]
        dz = rot[2][0]*d[0] + rot[2][1]*d[1] + rot[2][2]*d[2]
        rayFrom = [pos[0], pos[1], pos[2]+laser_height]
        rayTo = [rayFrom[0]+dx*laser_range, rayFrom[1]+dy*laser_range, rayFrom[2]]
        hit = p.raycast(rayFrom, rayTo)
        if hit[0] >= 0:
            dists.append(hit[2]*laser_range)
        else:
            dists.append(laser_range)
    return dists

# 仿真主循环
while True:
    dists = get_laser_distances()
    front = dists[0]
    right = dists[1]
    back = dists[2]
    left = dists[3]

    if front < 1.0:
        # 前方有障碍，原地右转
        left_speed = max_vel
        right_speed = -max_vel
    else:
        # 直行
        left_speed = max_vel
        right_speed = max_vel

    # 设置车轮速度
    p.setJointMotorControl2(car, wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=left_speed, force=force)
    p.setJointMotorControl2(car, wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=right_speed, force=force)
    p.setJointMotorControl2(car, wheel_joints[2], p.VELOCITY_CONTROL, targetVelocity=left_speed, force=force)
    p.setJointMotorControl2(car, wheel_joints[3], p.VELOCITY_CONTROL, targetVelocity=right_speed, force=force)

    p.stepSimulation()
    time.sleep(1/240)
