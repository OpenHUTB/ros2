import mujoco
import mujoco.viewer
import numpy as np

# 避障阈值
SAFE_FULL_DIST = 5.0   # 距离 > 5m: 全速前进
SAFE_SLOW_DIST = 2.0   # 2m < 距离 <= 5m: 减速; 距离 <= 2m: 急停

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    base_thrust = 12.0
    print("仿真启动, 无人机向前飞行, 检测障碍物距离")

    while viewer.is_running():
        # 获取无人机和障碍物之间距离
        drone_pos = data.body("drone").xpos
        obs_pos = data.geom("obstacle").xpos
        dist = np.linalg.norm(drone_pos - obs_pos)
        vx = data.qvel[0]

        # 三段避障逻辑
        if dist > SAFE_FULL_DIST:
            thrust = base_thrust
            print(f"距离{dist:.2f}m >5m, 全速前进")
        elif SAFE_SLOW_DIST < dist <= SAFE_FULL_DIST:
            thrust = base_thrust * 0.3
            print(f"距离{dist:.2f}m, 减速前进")
        else:
            if vx > 0.05:
                thrust = -base_thrust
                print(f"距离{dist:.2f}m <=2m, 紧急急停(制动中)!")
            else:
                thrust = 0
                data.qvel[0] = 0
                print(f"距离{dist:.2f}m <=2m, 已安全停止")

        # 获取无人机 body 索引并施加 X 方向推力
        drone_id = model.body("drone").id
        data.xfrc_applied[drone_id, 0] = thrust
        data.xfrc_applied[drone_id, 1] = 0
        data.xfrc_applied[drone_id, 2] = 0
        data.xfrc_applied[drone_id, 3] = 0
        data.xfrc_applied[drone_id, 4] = 0
        data.xfrc_applied[drone_id, 5] = 0

        mujoco.mj_step(model, data)
        viewer.sync()
