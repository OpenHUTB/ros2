#!/usr/bin/env python3
"""四轮独立悬架越野小车的仿真核心（不依赖 ROS）。

ROS 节点 headless 演示 都导入本模块，保证两条路径物理与控制完全一致。
"""
import os
import shutil
import sys

import numpy as np
import mujoco

from terrain import fill_hfield, elevation
from attitude_controller import OffroadController, wrap_angle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = "mujoco_offroad_suspension_robot_noetic"


def _model_dir():
    """定位 models/：源码树、devel/install 空间两种布局都要能跑。

    - 源码树：<pkg>/scripts/xxx.py          -> <pkg>/models
    - devel/install：<ws>/lib/<pkg>/xxx.py  -> <ws>/share/<pkg>/models
      （CMakeLists 把 models/ 装到 share，脚本与库模块装到 lib/<pkg>，所以模型目录要单独回退，
        否则 rosrun/roslaunch 起来会报 resource not found: models/suspension_robot.xml）
    - 需要改到别处时用环境变量 OFFROAD_MODEL_DIR 显式覆盖
    """
    cand = [
        os.environ.get("OFFROAD_MODEL_DIR", ""),
        os.path.join(ROOT, "models"),
        os.path.join(ROOT, os.pardir, "share", PKG, "models"),
    ]
    for c in cand:
        if c and os.path.isdir(c):
            return os.path.abspath(c)
    return os.path.join(ROOT, "models")


MODEL_DIR = _model_dir()
DEFAULT_XML = os.path.join(MODEL_DIR, "suspension_robot.xml")


def _writable(d):
    """目录存在且可写（创建测试文件后删掉）。"""
    try:
        os.makedirs(d, exist_ok=True)
        probe = os.path.join(d, ".write_probe")
        with open(probe, "w", encoding="utf-8") as f:
            f.write("")
        os.remove(probe)
        return True
    except OSError:
        return False


def repo_docs_img():
    """源码树里定位**仓库根**的 docs/img，装到工作空间后返回 None。

    ROOT 在源码树是 <repo>/src/ground/<pkg>，向上三级才是仓库根；装到 devel/install 后
    ROOT 是 <ws>/lib，再往上不会有 docs/index.md。所以按「同时存在 docs/index.md 与
    mkdocs.yml」向上逐级探测，而不是只看 <包目录>/docs 在不在。
    """
    p = os.path.abspath(ROOT)
    for _ in range(5):
        if (os.path.isfile(os.path.join(p, "docs", "index.md"))
                and os.path.isfile(os.path.join(p, "mkdocs.yml"))):
            img = os.path.join(p, "docs", "img")
            return img if os.path.isdir(img) else None
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    return None


def resolve_out_dir(explicit=None):
    """选输出目录：--out > $OFFROAD_OUT_DIR > 仓库根 docs/img > ~/.cache/<pkg>/output。

    源码树里默认写仓库根 docs/img（与文档页配图同一目录，重跑即原地更新）；装到
    <ws>/lib/<pkg>/ 后找不到仓库根，自动退到 ~/.cache/<pkg>/output。每个候选都做可写性探测。
    """
    cand = [explicit, os.environ.get("OFFROAD_OUT_DIR", "")]
    cand.append(repo_docs_img())
    cand.append(os.path.join(os.path.expanduser("~"), ".cache", PKG, "output"))
    cand.append(os.path.join(os.getcwd(), "offroad_output"))
    for c in cand:
        if c and _writable(c):
            return os.path.abspath(c)
    raise OSError("找不到可写的输出目录，请用 --out 指定")

# XML 里 <texture file="soil.png"> / "rock.png" 是程序生成的贴图，不入库
ASSET_FILES = ("soil.png", "rock.png")


def ensure_assets(xml_path=DEFAULT_XML):
    """贴图缺失时先调用 gen_model.write_textures() 生成，返回实际可用的 XML 路径。

    不入库是因为它们是 gen_model.py 的产物（512×512 程序噪声），换地形/配色只需重跑生成，
    而缺少时 muJoCo 会直接报 `resource not found: models/soil.png`，新手很容易卡在这里。

    安装到 <ws>/share/<pkg>/models 后该目录可能是只读的（rosdep/系统目录），此时把 XML 与
    贴图镜像到 ~/.cache/<pkg>/models 再加载，避免第一次 roslaunch 就往 share 里写文件。
    """
    model_dir = os.path.dirname(os.path.abspath(xml_path))
    missing = [n for n in ASSET_FILES if not os.path.isfile(os.path.join(model_dir, n))]
    if not missing:
        return xml_path
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    import gen_model  # noqa: PLC0415  同目录模块

    if _writable(model_dir):
        gen_model.write_textures(model_dir)
        return xml_path

    cache = os.path.join(os.path.expanduser("~"), ".cache", PKG, "models")
    os.makedirs(cache, exist_ok=True)
    for n in os.listdir(model_dir):
        src = os.path.join(model_dir, n)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(cache, n))
    gen_model.write_textures(cache)
    print("警告: %s 只读，贴图已生成到 %s，并用该目录的模型副本加载" % (model_dir, cache))
    return os.path.join(cache, os.path.basename(xml_path))


SUS_JOINTS = ["sus_j_fl", "sus_j_fr", "sus_j_rl", "sus_j_rr"]
WHEEL_JOINTS = ["drive_fl", "drive_fr", "drive_rl", "drive_rr"]


def quat_to_euler(q):
    """wxyz -> (roll, pitch, yaw)，与 ROS 欧拉角约定一致 (RPY, ZYX)。"""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class OffroadSim:
    def __init__(self, xml_path=DEFAULT_XML, target=(18.0, 2.0),
                 start=(-1.5, -1.0), tilt_assist_gain=0.0, settle_time=0.9):
        xml_used = ensure_assets(xml_path)
        self.model = mujoco.MjModel.from_xml_path(xml_used)
        self.data = mujoco.MjData(self.model)
        self.terrain_info = fill_hfield(self.model)

        self.target = target
        self.start = start
        self.ctrl = OffroadController(tilt_assist_gain=tilt_assist_gain)
        self.settle_time = settle_time
        self.dt = float(self.model.opt.timestep)

        self.sus_qpos_adr = [self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in SUS_JOINTS]
        self.wheel_dof_adr = [self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in WHEEL_JOINTS]

        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        d, m = self.data, self.model
        mujoco.mj_resetData(m, d)
        x0, y0 = self.start
        d.qpos[0] = x0
        d.qpos[1] = y0
        d.qpos[2] = float(elevation(x0, y0)) + 0.30   # 略高于地面，靠悬架落下来
        # 朝向目标
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        yaw0 = float(np.arctan2(self.target[1] - y0, self.target[0] - x0))
        d.qpos[3:7] = [np.cos(yaw0 / 2), 0.0, 0.0, np.sin(yaw0 / 2)]
        mujoco.mj_forward(m, d)
        # 静置落车，让悬架进入静平衡
        for _ in range(int(self.settle_time / self.dt)):
            d.ctrl[:] = 0.0
            mujoco.mj_step(m, d)
        self.ctrl.v_ref = 0.0

    # ------------------------------------------------------------------
    def state(self):
        d = self.data
        roll, pitch, yaw = quat_to_euler(d.qpos[3:7])
        # 车体前向速度（世界系速度投影到车头方向）
        vx, vy = d.qvel[0], d.qvel[1]
        v_body = float(vx * np.cos(yaw) + vy * np.sin(yaw))
        return {
            "t": float(d.time),
            "x": float(d.qpos[0]),
            "y": float(d.qpos[1]),
            "z": float(d.qpos[2]),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "v_body": v_body,
            "sus": [float(d.qpos[a]) for a in self.sus_qpos_adr],
            "wheel_w": [float(d.qvel[a]) for a in self.wheel_dof_adr],
            "terrain_h": float(elevation(d.qpos[0], d.qpos[1])),
            "gyro": [float(v) for v in d.sensordata[0:3]],
            "accel": [float(v) for v in d.sensordata[3:6]],
        }

    # ------------------------------------------------------------------
    def step(self):
        s = self.state()
        cmd = self.ctrl.command(s["x"], s["y"], s["yaw"], s["roll"], s["pitch"],
                                s["v_body"], self.target, self.dt)
        self.data.ctrl[0] = cmd["w_left"]
        self.data.ctrl[1] = cmd["w_right"]
        self.data.ctrl[2] = cmd["w_left"]
        self.data.ctrl[3] = cmd["w_right"]
        mujoco.mj_step(self.model, self.data)
        return cmd

    # ------------------------------------------------------------------
    def run(self, duration, on_sample=None, sample_dt=0.02, on_frame=None, frame_dt=1 / 20):
        """跑指定时长；on_sample 每个采样点回调，on_frame 按帧率回调（渲染用）。"""
        m, d = self.model, self.data
        t_end = d.time + duration
        # 从当前时刻起计，避免 reset() 静置阶段的时间被"补采样"（否则开头会多出几十个挤在一起的点）
        next_sample = d.time
        next_frame = d.time
        samples = []
        while d.time < t_end:
            cmd = self.step()
            if d.time >= next_sample:
                s = self.state()
                s["cmd"] = cmd
                samples.append(s)
                if on_sample:
                    on_sample(s)
                next_sample += sample_dt
            if d.time >= next_frame:
                if on_frame:
                    on_frame(d)
                next_frame += frame_dt
        return samples


# ----------------------------------------------------------------------
def pose_summary(samples, target):
    """把轨迹数据汇总为可核对的指标。"""
    arr = {k: np.array([s[k] for s in samples]) for k in
           ["x", "y", "z", "roll", "pitch", "yaw", "v_body", "terrain_h"]}
    sus = np.array([s["sus"] for s in samples])
    wheel_w = np.array([s["wheel_w"] for s in samples])
    v_cmd = np.array([s["cmd"]["v_cmd"] for s in samples])
    path = np.column_stack([arr["x"], arr["y"]])
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s_prog = float(seg.sum())
    final = float(np.hypot(target[0] - arr["x"][-1], target[1] - arr["y"][-1]))
    return {
        "frames": len(samples),
        "duration_s": float(samples[-1]["t"] - samples[0]["t"]),
        "path_length_m": s_prog,
        "mean_speed_mps": float(s_prog / max(1e-9, samples[-1]["t"] - samples[0]["t"])),
        "cmd_speed_mean_mps": float(v_cmd.mean()),
        "final_dist_to_target_m": final,
        "min_dist_to_target_m": float(np.min(np.hypot(target[0] - arr["x"], target[1] - arr["y"]))),
        "roll_rms_deg": float(np.degrees(np.sqrt((arr["roll"] ** 2).mean()))),
        "pitch_rms_deg": float(np.degrees(np.sqrt((arr["pitch"] ** 2).mean()))),
        "roll_max_deg": float(np.degrees(np.abs(arr["roll"]).max())),
        "pitch_max_deg": float(np.degrees(np.abs(arr["pitch"]).max())),
        "sus_travel_mm": [float(np.abs(sus[:, i]).max() * 1000) for i in range(4)],
        "sus_travel_pp_mm": [float((sus[:, i].max() - sus[:, i].min()) * 1000) for i in range(4)],
        "wheel_speed_max_rps": float(np.abs(wheel_w).max()),
        "clearance_min_m": float(np.min(arr["z"] - arr["terrain_h"])),
    }
