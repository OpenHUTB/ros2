# -*- coding: utf-8 -*-
"""Shadow Hand 右手灵巧手：手指开合摆动（MuJoCo，不依赖 Gazebo）。

运行：python3 main.py            # 输出 output/shadow_hand_grasp.mp4
检查取景：python3 main.py --preview
"""
import numpy as np

# 按包导入：源码树直跑由模块根目录的 main.py 先把 src/ 加入 sys.path 后转发，
# catkin 安装后由 setup.py / catkin 环境提供包路径。
from shadow_hand_mujoco.common import setup_gl, load_with_camera, run_sim, parse_args

# 固定相机：上前方 3/4 俯视，对准手掌中心（手平放于地面、沿 +x 展开）
CAM = dict(name="view0", pos=(0.35, -0.45, 0.35),
           xyaxes=(0.936, 0.352, 0, -0.201, 0.533, 0.822), fovy=45)

ROBOT = "shadow_hand"
ENTRY = "scene_right"


def make_controller(model):
    import mujoco

    # 各执行器驱动关节的初始角度作为基准
    d0 = mujoco.MjData(model)
    names = [model.actuator(i).name for i in range(model.nu)]
    base = np.zeros(model.nu)
    for i in range(model.nu):
        jid = model.actuator_trnid[i, 0]
        qadr = model.jnt_qposadr[jid]
        base[i] = d0.qpos[qadr]

    def amp(name):
        if name.endswith("J3") or name.endswith("J4") or name.endswith("J5"):
            return 0.55   # 中/远端屈曲关节，幅度最大
        if "THJ" in name:
            return 0.40   # 拇指
        if "WRJ" in name:
            return 0.22   # 手腕
        return 0.30       # 其余（近端/掌指）

    def phase(name):
        if "LFJ" in name:
            return np.pi        # 小指与其余三指反相
        if "THJ" in name:
            return np.pi / 2    # 拇指错相
        if "WRJ" in name:
            return np.pi / 4
        return 0.0

    freq = 0.5  # 开合频率 Hz（2 秒一个循环）

    def ctrl(data, t):
        for i in range(model.nu):
            data.ctrl[i] = base[i] + amp(names[i]) * np.sin(
                2.0 * np.pi * freq * t + phase(names[i]))

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
        run_sim(model, ctrl, duration=8.0, fps=30, video_path=args.out or "output/shadow_hand_grasp.mp4")


if __name__ == "__main__":
    main()
