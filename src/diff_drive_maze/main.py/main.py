import mujoco
import mujoco.viewer
import time

# 加载模型
model = mujoco.MjModel.from_xml_path("diff_drive_maze.xml")
data = mujoco.MjData(model)

# 打开可视化窗口
viewer = mujoco.viewer.launch_passive(model, data)

# 开环控制序列：(左轮速度, 右轮速度, 步数)
sequence = [
    (1.0, 1.0, 3000),   # 直行
    (-1.0, 1.0, 250),   # 左转90度
    (1.0, 1.0, 2500),   # 直行
    (1.0, -1.0, 265),   # 右转90度
    (1.0, 1.0, 3000),   # 直行到终点
]

step_idx = 0
step_count = 0
running = True

while running and viewer.is_running():
    if step_idx < len(sequence):
        left, right, total = sequence[step_idx]
        data.ctrl[0] = left
        data.ctrl[1] = right
        step_count += 1
        if step_count >= total:
            step_idx += 1
            step_count = 0
    else:
        data.ctrl[0] = 0
        data.ctrl[1] = 0

    # 碰撞检测
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1 = model.geom(contact.geom1).name
        geom2 = model.geom(contact.geom2).name
        if "wall" in geom1 or "wall" in geom2:
            print(f"撞墙了！碰撞geom: {geom1}, {geom2}")
            data.ctrl[0] = 0
            data.ctrl[1] = 0
            running = False
            break
        if "goal" in geom1 or "goal" in geom2:
            print(f"到达终点！碰撞geom: {geom1}, {geom2}")
            data.ctrl[0] = 0
            data.ctrl[1] = 0
            running = False
            break

    mujoco.mj_step(model, data)
    viewer.sync()
    time.sleep(0.005)

print(f"最终位置: x={data.qpos[0]:.2f}, y={data.qpos[1]:.2f}")
viewer.close()
