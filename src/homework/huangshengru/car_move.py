import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf", [0,0,0.2])
wheel_joints = [2,3,6,7]

cycle_time = 6
start_time = time.time()

while True:
    t = (time.time() - start_time) % cycle_time
    if t < 3:
        v = 8      # 0~3s 前进
    elif t < 6:
        v = -8     # 3~6s 后退

    for joint in wheel_joints:
        p.setJointMotorControl2(robotId, joint, p.VELOCITY_CONTROL, targetVelocity=v, force=20)

    p.stepSimulation()
    time.sleep(1/240)
