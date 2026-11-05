# -*- coding: utf-8 -*-
"""Franka Emika Panda 七轴机械臂：关节轨迹运动 + 夹爪开合（MuJoCo，不依赖 Gazebo）。

运行：python3 main.py            # 输出 output/panda_motion.mp4
检查取景：python3 main.py --preview
"""
import numpy as np

# 按包导入：源码树直跑由模块根目录的 main.py 先把 src/ 加入 sys.path 后转发，
# catkin 安装后由 setup.py / catkin 环境提供包路径。
from panda_mujoco.common import setup_gl, load_with_camera, run_sim, parse_args

# 固定相机：前侧 3/4 视角，对准手臂伸展中心 (0.3, 0, 0.5)
CAM = dict(name="view0", pos=(0.9, -1.2, 0.55),
           xyaxes=(0.893, 0.451, 0, -0.018, 0.036, 0.999), fovy=40)

ROBOT = "franka_emika_panda"
ENTRY = "scene"


def make_controller(model):
    import mujoco

    scratch = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, scratch, 0)
    home = scratch.qpos[:7].copy()  # joint1..joint7（臂），无浮动基座

    # 目标位姿 = home + 增量，四段循环（上抬 -> 前伸 -> 侧摆 -> 归位）
    deltas = [
        np.zeros(7),
        np.array([0.0, 0.7, 0.15, 1.2, 0.0, 0.8, 0.0]),
        np.array([0.5, 0.4, 0.25, 0.6, -0.4, 0.3, 0.4]),
        np.zeros(7),
    ]
    poses = [home + d for d in deltas]
    seg_dur = 2.0  # 每段 2 秒

    g0, g1 = model.actuator(model.nu - 1).ctrlrange  # 夹爪开合范围

    def ctrl(data, t):
        seg = int(t / seg_dur) % 4
        lt = (t % seg_dur) / seg_dur
        s = 0.5 - 0.5 * np.cos(np.pi * lt)  # 余弦缓动
        target = poses[seg] * (1.0 - s) + poses[(seg + 1) % 4] * s
        data.ctrl[:7] = target
        # 夹爪：8 秒一个开合周期
        open_ratio = 0.5 - 0.5 * np.cos(2.0 * np.pi * t / 8.0)
        data.ctrl[7] = g0 + (g1 - g0) * open_ratio

    return ctrl


def build_model():
    """加载模型（主流程与单元测试共用）。"""
    return load_with_camera(ROBOT, ENTRY, CAM)


def main():
    args = parse_args()
    gl = setup_gl()
    model = build_model()
    print(f"渲染后端: {gl} | {ROBOT} ({model.nq} qpos, {model.nu} actuators)")
    ctrl = make_controller(model)
    if args.preview:
        run_sim(model, ctrl, duration=1.6, fps=30, preview_dir="output/preview")
    else:
        run_sim(model, ctrl, duration=8.0, fps=30, video_path=args.out or "output/panda_motion.mp4")


if __name__ == "__main__":
    main()
