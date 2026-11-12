"""开发期集成测试：行走 + 激光建图 + 扫描匹配 SLAM。"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from humanoid_mujoco_slam_nn_nav.gait import GaitParams, Walker  # noqa: E402
from humanoid_mujoco_slam_nn_nav.mapping import OccupancyGrid, Slam  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sensors import Lidar, Odometry  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim  # noqa: E402


def main(seconds=18.0, scene="warehouse"):
    sim = HumanoidSim(scene=scene, seed=0)
    sx, sy, syaw = sim.layout.start
    gp = GaitParams(pelvis_mode="feet", pelvis_height=0.70, duty=0.78, period=1.15,
                    side_shift=0.02, foot_spread=0.07, step_height=0.08,
                    capture_gain=0.35, capture_max=0.08, com_kp=1.0, com_kd=0.1,
                    com_max=0.04, roll_gain=0.0, pitch_gain=0.0, ankle_kp=0.0, ankle_kd=0.0)
    walker = Walker(gp, sx, sy, syaw)
    walker.command(0.25, 0.0)
    sim.reset()

    lidar = Lidar(sim.model, n_beams=180, max_range=8.0)
    grid = OccupancyGrid(size_x=10, size_y=8, res=0.05)
    slam = Slam(grid)
    slam.reset((sx, sy, syaw))
    odom = Odometry()
    odom.reset(sx, sy, syaw)
    prev_odom = (sx, sy, syaw)
    rng = np.random.default_rng(0)

    dt = 0.01
    t = 0.0
    last_scan_t = -1.0
    ate = []
    scans = 0
    while t < seconds and not sim.is_fallen():
        st = sim.state()
        feet = {"l": tuple(sim.ankle_pos("l")[:2]), "r": tuple(sim.ankle_pos("r")[:2])}
        q = walker.update(dt, actual_pos=st["pos"], actual_yaw=st["yaw"],
                          roll=st["roll"], pitch=st["pitch"], vel=st["vel"], feet_xy=feet)
        sim.set_targets(q)
        sim.step(int(round(dt / sim.model.opt.timestep)))
        t += dt

        if t - last_scan_t > 0.1:
            last_scan_t = t
            ranges = lidar.scan(sim.data, rng)
            o = odom.update(st["vel"][0], st["vel"][1], 0.0, 0.1, rng)
            dx = o["x"] - prev_odom[0]
            dy = o["y"] - prev_odom[1]
            dyaw = math.atan2(math.sin(o["yaw"] - prev_odom[2]),
                              math.cos(o["yaw"] - prev_odom[2]))
            prev_odom = (o["x"], o["y"], o["yaw"])
            pose = slam.update((dx, dy, dyaw), ranges, lidar.angles)
            scans += 1
            ate.append(math.hypot(pose[0] - st["pos"][0], pose[1] - st["pos"][1]))

    st = sim.state()
    print(f"scans={scans} t={t:.1f} fallen={sim.is_fallen()} travel={st['pos'][0]-sx:+.2f}")
    print(f"map coverage={grid.coverage():.3f}  ATE rms={np.sqrt(np.mean(np.square(ate))):.3f} m")
    print(f"final pose est={np.round(slam.pose,2)} gt={np.round([st['pos'][0], st['pos'][1], st['yaw']],2)}")

    # 地图与真值障碍对比
    occ = grid.occupied(0.65)
    hits = 0
    tot = 0
    for o in sim.layout.obstacles:
        cx, cy = grid.world_to_cell(o.x, o.y)
        r = int(0.15 / grid.res)
        y0, y1 = max(0, cy - r), min(grid.ny, cy + r + 1)
        x0, x1 = max(0, cx - r), min(grid.nx, cx + r + 1)
        if y1 > y0 and x1 > x0:
            hits += int(occ[y0:y1, x0:x1].any())
            tot += 1
    print(f"obstacle recall={hits}/{tot}")


if __name__ == "__main__":
    main()
