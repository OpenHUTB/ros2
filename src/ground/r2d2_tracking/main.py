import pybullet as p
import pybullet_data
import time
import math

# 连接 PyBullet 可视化仿真窗口
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载地面、R2D2 小车与目标球
p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", [0, 0, 0.2])
target = p.loadURDF("sphere_small.urdf", [2, 0, 0.2])

# 开启实时仿真，仿真自动推进
p.setRealTimeSimulation(1)

# R2D2 车轮关节（依据 r2d2.urdf：joint 2,3 为右侧轮，6,7 为左侧轮）
left_wheels = [6, 7]
right_wheels = [2, 3]
force = 50
turn_gain = 8.0

t = 0.0
while True:
    # 目标球沿圆形轨迹持续移动（球速约 0.36 m/s）
    t += 0.02
    tx = 1.2 * math.cos(t * 0.3)
    ty = 1.2 * math.sin(t * 0.3)
    p.resetBasePositionAndOrientation(target, [tx, ty, 0.2],
                                      p.getQuaternionFromEuler([0, 0, 0]))

    # 球运动方向（切线方向），用于计算球后方的跟随点
    bdir = math.atan2(1.2 * 0.3 * math.cos(t * 0.3),
                      -1.2 * 0.3 * math.sin(t * 0.3))

    # 小车位置与朝向（R2D2 车头朝 +Y 方向）
    pos, orn = p.getBasePositionAndOrientation(robot)
    yaw = p.getEulerFromQuaternion(orn)[2]
    head_angle = math.pi / 2 + yaw   # 车头方向在世界系中的角度

    # 跟随点：球后方 0.5m（小车跟在球后面，不重合不自转）
    fx = tx - 0.5 * math.cos(bdir)
    fy = ty - 0.5 * math.sin(bdir)
    dx = fx - pos[0]
    dy = fy - pos[1]
    target_angle = math.atan2(dy, dx)

    # 转向误差（归一化到 [-pi, pi]）
    err = target_angle - head_angle
    while err > math.pi:
        err -= 2 * math.pi
    while err < -math.pi:
        err += 2 * math.pi

    # 速度按距离分级：远则快追，近则减速保持，稳定跟在球后面
    dist = math.hypot(dx, dy)
    if dist > 1.2:
        base_speed = 16.0
    elif dist < 0.4:
        base_speed = 6.0
    else:
        base_speed = 11.0

    # 差速控制（物理校准：左轮快->右转；目标在左时右轮加速实现左转）
    left_speed = base_speed - turn_gain * err
    right_speed = base_speed + turn_gain * err

    for joint in left_wheels:
        p.setJointMotorControl2(robot, joint, p.VELOCITY_CONTROL,
                                targetVelocity=left_speed, force=force)
    for joint in right_wheels:
        p.setJointMotorControl2(robot, joint, p.VELOCITY_CONTROL,
                                targetVelocity=right_speed, force=force)

    time.sleep(0.02)
