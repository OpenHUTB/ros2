# -*- coding: utf-8 -*-
"""宇树 Go2 四足机器人 MuJoCo 仿真模块（OpenHUTB/ros2 仓库规范模块）。

原理：
  Go2 的 12 个执行器是力矩电机（ctrlrange ±23.7 N·m），需要自己写 PD 控制器。
  本脚本用「平面逆运动学」把每条腿的足端画一个椭圆轨迹（摆动相抬脚、支撑相贴地），
  再由 PD 力矩把关节拉到 IK 解出的目标角，机器人即可稳定向前小跑。

运行：cd src/ground/go2_mujoco && python3 main.py   # 输出 output/go2_trot.mp4
检查取景：python3 main.py --preview
"""
import numpy as np

# 按包导入：源码树直跑由模块根目录的 main.py 先把 src/ 加入 sys.path 后转发，
# catkin 安装后由 setup.py / catkin 环境提供包路径。
from go2_mujoco.common import setup_gl, load_with_camera, run_sim, parse_args

# 跟踪相机：正侧方跟随机身 base，距离拉远以容纳步态中的轻微转向
CAM = dict(name="view0", pos=(0, -2.4, 0.6), xyaxes=(1, 0, 0, 0, 0.28, 0.96),
           fovy=38, mode="track", target="base")

ROBOT = "unitree_go2"
ENTRY = "scene"

KP, KD = 20.0, 0.5          # PD 增益（与 legged_gym 对 Go2 的常用配置一致）
LEGS = ["FL", "FR", "RL", "RR"]
TROT = {"FL": 0.0, "RR": 0.0, "FR": np.pi, "RL": np.pi}   # 对角小跑配对

FREQ = 1.1    # 步频 Hz
A_X = 0.10    # 足端前后摆动幅度 m
LIFT = 0.042  # 摆动相抬脚高度 m


def make_controller(model):
    import mujoco

    # 各关节 home 目标角（从 keyframe 取）
    home = np.array([float(model.keyframe(0).qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]])
                     for i in range(model.nu)])
    idx = {model.actuator(i).name: i for i in range(model.nu)}

    # 每条腿的平面几何（在 base 系下）：大腿长 L1、小腿长 L2、home 足端相对髋 (rx, rz)
    geom = {}
    for leg in LEGS:
        hip = model.body(f"{leg}_hip").pos
        thigh = model.body(f"{leg}_thigh").pos
        calf = model.body(f"{leg}_calf").pos
        L1 = float(np.linalg.norm(thigh - hip))
        L2 = float(np.linalg.norm(calf - thigh))
        rx, rz = float(calf[0] - hip[0]), float(calf[2] - hip[2])
        geom[leg] = (L1, L2, rx, rz)

    def ik(L1, L2, fx, fz):
        """平面两连杆 IK：返回 (大腿角, 膝角)，与 home 同基准（符号自洽）。"""
        r = np.hypot(fx, fz)
        r = min(max(r, abs(L1 - L2) + 1e-3), L1 + L2 - 1e-3)
        knee = np.arccos(np.clip((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2), -1, 1))
        a1 = np.arctan2(fx, -fz)
        a2 = np.arccos(np.clip((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r), -1, 1))
        return a1 - a2, knee

    ang_home = {leg: ik(*geom[leg][:2], geom[leg][2], geom[leg][3]) for leg in LEGS}

    def ctrl(data, t):
        for leg in LEGS:
            L1, L2, rx, rz = geom[leg]
            p = 2.0 * np.pi * FREQ * t + TROT[leg]
            fx = rx + A_X * np.cos(p)
            fz = rz + (LIFT * np.sin(p) if np.sin(p) > 0 else 0.0)   # 摆动相抬脚
            ta, kn = ik(L1, L2, fx, fz)
            ta_h, kn_h = ang_home[leg]
            qd_thigh = home[idx[f"{leg}_thigh"]] + (ta - ta_h)
            qd_calf = home[idx[f"{leg}_calf"]] + (kn - kn_h)
            qd_hip = home[idx[f"{leg}_hip"]]

            # PD 力矩（大腿/小腿），髋部保持 home 并加轻阻尼
            for joint, qd in (("thigh", qd_thigh), ("calf", qd_calf), ("hip", qd_hip)):
                i = idx[f"{leg}_{joint}"]
                jid = model.actuator_trnid[i, 0]
                q = data.qpos[model.jnt_qposadr[jid]]
                qv = data.qvel[model.jnt_dofadr[jid]]
                data.ctrl[i] = float(np.clip(KP * (qd - q) - KD * qv,
                                            *model.actuator(i).ctrlrange))

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
        run_sim(model, ctrl, duration=6.0, fps=30, video_path=args.out or "output/go2_trot.mp4")


if __name__ == "__main__":
    main()
