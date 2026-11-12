#!/usr/bin/env python3
"""mujoco_car_follow：引导车在 ±3 m 内往返走 S 形轨迹，
跟随车用「前馈 + PD」控制保持 0.5 m 的真实车心距。

默认打开 MuJoCo 窗口；`--headless` 不依赖窗口/Xvfb，直接跑完并输出
数值统计与 CSV，便于 CI 或无显示环境复现 README 中的结果。
"""
import argparse
import csv
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

TARGET = 0.5              # 纵向目标车心距（m），按两车车身中心的世界坐标计算
LEAD_X_FORCE = 0.2        # 引导车纵向推力（N），稳态速度 = 0.2/0.3 = 0.667 m/s
LEAD_Y_AMP = 0.3          # 引导车横向正弦振幅（m）
LEAD_Y_FREQ = 2.0         # 引导车横向正弦角频率（rad/s）

KP_X, KD_X = 25.0, 2.0    # 纵向 PD 增益
KP_Y, KD_Y = 15.0, 1.5    # 横向 PD 增益
FMAX = 1.0                # 单个电机输出限幅（N）
X_LIMIT = 3.0             # 引导车往返边界（m）
FF_RATE = 0.02            # 前馈力每步最大变化量（N），折返时平滑过渡
LOG_INTERVAL = 0.5        # 日志打印间隔（s），避免每秒 100 行刷屏
WARMUP_SECONDS = 3.0      # 统计时去掉的启动过渡段（s）


def resolve_scene_path():
    """优先从 ROS 包 share 目录查找 scene.xml，直接运行脚本时回退到脚本目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("mujoco_car_follow"), "scene.xml")
        if os.path.exists(path):
            return path
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (os.path.join(here, "scene.xml"),
                      os.path.join(here, os.pardir, "scene.xml")):
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
    return os.path.join(here, os.pardir, "scene.xml")


def joint_index(model, name):
    """返回 (qpos 地址, dof 地址)。"""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return model.jnt_qposadr[jid], model.jnt_dofadr[jid]


def clamp(value, limit=FMAX):
    return max(-limit, min(limit, value))


def parse_args(argv=None):
    """命令行参数：默认行为不变，只有显式 --headless 才跳过窗口。"""
    parser = argparse.ArgumentParser(description="MuJoCo 双车跟车仿真")
    parser.add_argument("--headless", action="store_true",
                        help="不打开 MuJoCo 窗口，直接跑完并输出统计")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="无头模式运行时长（秒），默认 60")
    parser.add_argument("--out", default=None,
                        help="把逐次采样写入 CSV 的路径（可选）")
    return parser.parse_args(argv)


def summarize(dx_samples, dy_samples, log_interval=LOG_INTERVAL):
    """计算去掉启动过渡段后的距离统计。"""
    dx = np.asarray(dx_samples, dtype=float)
    dy = np.asarray(dy_samples, dtype=float)
    warmup = max(1, int(round(WARMUP_SECONDS / log_interval)))
    if len(dx) <= warmup:
        warmup = 0
    dx_eval = dx[warmup:]
    dy_eval = dy[warmup:]
    return {
        "samples": int(len(dx)),
        "distance_mean": float(dx_eval.mean()) if len(dx_eval) else float("nan"),
        "distance_max_error": float(np.abs(dx_eval - TARGET).max())
        if len(dx_eval) else float("nan"),
        "lateral_mean": float(dy_eval.mean()) if len(dy_eval) else float("nan"),
        "lateral_max_error": float(np.abs(dy_eval).max())
        if len(dy_eval) else float("nan"),
    }


def write_csv(path, rows):
    """写逐次采样：time, distance, lateral, lead_x, lead_y, follow_x, follow_y。"""
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "time_s", "distance_m", "lateral_m",
            "lead_x_m", "lead_y_m", "follow_x_m", "follow_y_m",
        ])
        writer.writerows(rows)


def main(argv=None):
    args = parse_args(argv)
    model = mujoco.MjModel.from_xml_path(resolve_scene_path())
    data = mujoco.MjData(model)

    # 关节地址
    q_lead_x, v_lead_x = joint_index(model, "lead_x")
    q_lead_y, v_lead_y = joint_index(model, "lead_y")
    q_follow_x, v_follow_x = joint_index(model, "follow_x")
    q_follow_y, v_follow_y = joint_index(model, "follow_y")

    # body 的初始偏移：slide 关节 qpos 是相对 body 初始位置的位移，
    # 因此世界坐标 = body.pos + qpos（这是之前距离算少的根因）
    lead_x0 = model.body("lead_car").pos[0]
    lead_y0 = model.body("lead_car").pos[1]
    follow_x0 = model.body("follow_car").pos[0]
    follow_y0 = model.body("follow_car").pos[1]

    t = 0.0
    next_log = 0.0
    direction = 1.0                 # 引导车 x 方向：+1 前进，-1 折返
    feedforward = 0.0               # 平滑后的纵向前馈力，折返时不突变
    dx_samples, dy_samples = [], []
    rows = []

    def step_once(elapsed):
        """执行一步控制律并记录采样；返回当前 (dx, dy)。"""
        nonlocal direction, feedforward, next_log

        # ---------- 引导车：±3 m 内往返 + 横向正弦，走 S 形 ----------
        if data.qpos[q_lead_x] > X_LIMIT:
            direction = -1.0
        elif data.qpos[q_lead_x] < -X_LIMIT:
            direction = 1.0
        data.ctrl[0] = LEAD_X_FORCE * direction
        data.ctrl[1] = LEAD_Y_AMP * np.cos(LEAD_Y_FREQ * elapsed)

        # ---------- 世界坐标（含 body 初始偏移） ----------
        lead_x = lead_x0 + data.qpos[q_lead_x]
        lead_y = lead_y0 + data.qpos[q_lead_y]
        follow_x = follow_x0 + data.qpos[q_follow_x]
        follow_y = follow_y0 + data.qpos[q_follow_y]
        vfx = data.qvel[v_follow_x]
        vfy = data.qvel[v_follow_y]

        dx = lead_x - follow_x      # 真实纵向车心距
        dy = lead_y - follow_y      # 真实横向偏差

        # ---------- 跟随车：前馈抵消阻尼负载 + PD 消除稳态误差 ----------
        # 前馈取引导车当前推力并做变化率限幅，折返瞬间不会引起冲击
        feedforward += max(-FF_RATE, min(FF_RATE, data.ctrl[0] - feedforward))
        data.ctrl[2] = clamp(feedforward + KP_X * (dx - TARGET) - KD_X * vfx)
        data.ctrl[3] = clamp(data.ctrl[1] + KP_Y * dy - KD_Y * vfy)

        if elapsed >= next_log:
            next_log += LOG_INTERVAL
            dx_samples.append(dx)
            dy_samples.append(dy)
            rows.append((elapsed, dx, dy, lead_x, lead_y, follow_x, follow_y))
            print(f"[t={elapsed:6.2f}s] 真实车心距={dx:.3f} m  "
                  f"横向残差={dy:+.3f} m | "
                  f"引导({lead_x:+.3f},{lead_y:+.3f}) "
                  f"跟随({follow_x:+.3f},{follow_y:+.3f})", flush=True)
        return dx, dy

    if args.headless:
        steps = max(1, int(round(args.duration / model.opt.timestep)))
        for _ in range(steps):
            t += model.opt.timestep
            step_once(t)
            mujoco.mj_step(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                t += model.opt.timestep
                step_once(t)
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)

    stats = summarize(dx_samples, dy_samples)
    if stats["samples"]:
        print("\n===== 统计（去掉前 %.0f s 过渡段） =====" % WARMUP_SECONDS)
        print(f"纵向车心距：均值 {stats['distance_mean']:.3f} m，"
              f"最大偏差 {stats['distance_max_error']:.3f} m")
        print(f"横向残差：均值 {stats['lateral_mean']:+.3f} m，"
              f"最大偏差 {stats['lateral_max_error']:.3f} m")
        print(f"采样点数：{stats['samples']}")
    if args.out:
        write_csv(args.out, rows)
        print(f"CSV 已写入：{os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
