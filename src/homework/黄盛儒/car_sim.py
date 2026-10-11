import pybullet as p
import pybullet_data
import time

# 打开仿真窗口
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载地面
planeId = p.loadURDF("plane.urdf")

# 加载R2D2小车
carStartPos = [0, 0, 0.1]
carStartOrn = p.getQuaternionFromEuler([0,0,0])
car = p.loadURDF("r2d2.urdf", carStartPos, carStartOrn)

# 四个车轮关节
wheel_ids = [2,3,6,7]
max_speed = 6
force = 10

# 状态标记：前进/后退
state = "forward"
timer = 0
duration = 2.0 # 前进2秒，后退2秒

while True:
    if state == "forward":
        # 前进，四轮同方向正速度
        for wheel in wheel_ids:
            p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=max_speed, force=force)
        timer += 1/240
        if timer >= duration:
            state = "backward"
            timer = 0
    elif state == "backward":
        # 后退，负速度
        for wheel in wheel_ids:
            p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=-max_speed, force=force)
        timer += 1/240
        if timer >= duration:
            state = "forward"
            timer = 0

    p.stepSimulation()
    time.sleep(1/240)

