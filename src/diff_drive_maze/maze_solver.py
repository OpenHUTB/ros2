import os
import time
import mujoco
import numpy as np

XML_PATH = os.path.join(os.path.dirname(__file__), "diff_drive_maze.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.3

def check_collision():
    hit_wall = False
    hit_goal = False
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1 = model.geom(contact.geom1).name
        geom2 = model.geom(contact.geom2).name
        if 'wall' in geom1 or 'wall' in geom2:
            hit_wall = True
        if 'goal' in geom1 or 'goal' in geom2:
            hit_goal = True
    return hit_wall, hit_goal

def stop_robot():
    data.ctrl[0] = 0
    data.ctrl[1] = 0

def run(headless=False):
    step = 0
    finished = False
    
    if not headless:
        from mujoco import viewer
        v = viewer.launch_passive(model, data)
        time.sleep(1)
    
    print("阶段1：往右走")
    for _ in range(800):
        hit_wall, hit_goal = check_collision()
        if hit_wall:
            print("撞到墙了！失败")
            stop_robot()
            finished = True
            break
        if hit_goal:
            print("到达终点！成功！")
            stop_robot()
            finished = True
            break
        data.ctrl[0] = 0.3 / WHEEL_RADIUS
        data.ctrl[1] = 0.3 / WHEEL_RADIUS
        mujoco.mj_step(model, data)
        step += 1
        if not headless:
            v.sync()
            time.sleep(0.005)
    
    if not finished:
        print("阶段2：左转")
        for _ in range(250):
            hit_wall, hit_goal = check_collision()
            if hit_wall:
                print("撞到墙了！失败")
                stop_robot()
                finished = True
                break
            if hit_goal:
                print("到达终点！成功！")
                stop_robot()
                finished = True
                break
            data.ctrl[0] = -5.0
            data.ctrl[1] = 5.0
            mujoco.mj_step(model, data)
            step += 1
            if not headless:
                v.sync()
                time.sleep(0.005)
    
    if not finished:
        print("阶段3：往上走")
        for _ in range(3000):
            hit_wall, hit_goal = check_collision()
            if hit_wall:
                print("撞到墙了！失败")
                stop_robot()
                finished = True
                break
            if hit_goal:
                print("到达终点！成功！")
                stop_robot()
                finished = True
                break
            data.ctrl[0] = 0.3 / WHEEL_RADIUS
            data.ctrl[1] = 0.3 / WHEEL_RADIUS
            mujoco.mj_step(model, data)
            step += 1
            if not headless:
                v.sync()
                time.sleep(0.005)
    
    if not finished:
        print("阶段4：右转")
        for _ in range(265):
            hit_wall, hit_goal = check_collision()
            if hit_wall:
                print("撞到墙了！失败")
                stop_robot()
                finished = True
                break
            if hit_goal:
                print("到达终点！成功！")
                stop_robot()
                finished = True
                break
            data.ctrl[0] = 5.0
            data.ctrl[1] = -5.0
            mujoco.mj_step(model, data)
            step += 1
            if not headless:
                v.sync()
                time.sleep(0.005)
    
    if not finished:
        print("阶段5：往右走")
        for _ in range(2500):
            hit_wall, hit_goal = check_collision()
            if hit_wall:
                print("撞到墙了！失败")
                stop_robot()
                finished = True
                break
            if hit_goal:
                print("到达终点！成功！")
                stop_robot()
                finished = True
                break
            data.ctrl[0] = 0.3 / WHEEL_RADIUS
            data.ctrl[1] = 0.3 / WHEEL_RADIUS
            mujoco.mj_step(model, data)
            step += 1
            if not headless:
                v.sync()
                time.sleep(0.005)
    
    x = data.body('robot').xpos[0]
    y = data.body('robot').xpos[1]
    print(f"结束！最终位置: ({x:.2f}, {y:.2f})")
    
    if not headless:
        while v.is_running():
            v.sync()
            time.sleep(0.01)

if __name__ == "__main__":
    import sys
    headless = "--headless" in sys.argv
    run(headless=headless)
