import mujoco as mj
from mujoco import viewer
import numpy as np
import time

# 加载模型
model = mj.MjModel.from_xml_path("/home/hjy/diff_drive.xml")
data = mj.MjData(model)

# 目标点
GOAL_X = 3.0
GOAL_Y = 2.0

# 控制参数
LINEAR_SPEED = 0.08
ANGULAR_SPEED = 0.1
K_ANGULAR = 0.5  # 比例系数：直行时也保留小的航向修正

def get_car_pose():
    """获取小车位置和朝向"""
    car_x = data.qpos[0]
    car_y = data.qpos[1]
    car_yaw = data.qpos[5]  # yaw 角
    return car_x, car_y, car_yaw

def get_goal_angle(car_x, car_y, car_yaw):
    """计算机器人到目标点的角度差"""
    dx = GOAL_X - car_x
    dy = GOAL_Y - car_y
    goal_angle = np.arctan2(dy, dx)
    angle_diff = goal_angle - car_yaw
    # 归一化到 [-pi, pi]
    angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
    return angle_diff

def get_distance_to_goal(car_x, car_y):
    """计算机器人到目标点的距离"""
    return np.sqrt((GOAL_X - car_x)**2 + (GOAL_Y - car_y)**2)

# 运行仿真
viewer = viewer.launch_passive(model, data)

step = 0
reached = False

while viewer.is_running():
    car_x, car_y, car_yaw = get_car_pose()
    angle_diff = get_goal_angle(car_x, car_y, car_yaw)
    dist_to_goal = get_distance_to_goal(car_x, car_y)

    # 到达目标点
    if dist_to_goal < 0.15:
        data.ctrl[0] = 0
        data.ctrl[1] = 0
        reached = True
        print(f"到达目标点！步数: {step}")
        break

    # 分段比例控制（解决极限环问题）
    if abs(angle_diff) > 0.5:
        # 角度差很大：原地转
        left_speed = 0.0
        right_speed = -ANGULAR_SPEED if angle_diff > 0 else ANGULAR_SPEED
    else:
        # 角度差不大：边走边转（比例控制，直行时也保留小修正）
        left_speed = LINEAR_SPEED - K_ANGULAR * angle_diff
        right_speed = LINEAR_SPEED + K_ANGULAR * angle_diff

    # 设置轮子速度
    data.ctrl[0] = left_speed
    data.ctrl[1] = right_speed

    mj.mj_step(model, data)
    viewer.sync()
    step += 1
    time.sleep(0.001)

if not reached:
    print(f"未到达目标，最终距离: {dist_to_goal:.2f} m")
print("仿真结束")
