import time
import random

def get_front_distance():
    return round(random.uniform(0.2, 1.5), 2)

def robot_control(distance):
    if distance > 0.5:
        return "直行，线速度0.3m/s"
    else:
        return "转弯避障，角速度0.8rad/s"

if __name__ == "__main__":
    print("Win11轮式机器人核心避障启动")
    for i in range(15):
        dist = get_front_distance()
        action = robot_control(dist)
        print(f"第{i+1}步：前方距离{dist}m → {action}")
        time.sleep(0.6)
    print("核心控制流程结束")