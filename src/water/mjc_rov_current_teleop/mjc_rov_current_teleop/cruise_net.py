"""
cruise_net.py —— MLP 巡航策略（神经网络成分）。

为什么用行为克隆（BC）而不是直接上 PPO：
  作业要求"算法必须是神经网络"且要在无 GPU 虚拟机上几分钟内训出权重。
  BC 用脚本专家（waypoint P 控制器）采一批数据，MLP 回归出 (误差->速度指令)
  的映射，CPU 上 <1 分钟收敛，且权重只有 ~5k 参数，可复现、可对比。

输入特征（6 维）：
  [ex, ey, eyaw, vx, vy, wz]   —— 位置/航向误差 + 当前速度
输出（2 维）：
  [vx_world, vy_world]       —— 世界系期望速度，直接作为速度指令下发
"""
import json
import time
from pathlib import Path

import numpy as np

import torch
import torch.nn as nn

from . import common


# 专家航点（池内绕圈，z=0.5 为水柱中层，与 ROV 初始高度一致）
EXPERT_WAYPOINTS = np.array([
    [2.0, 0.0, 0.5],
    [0.0, 2.0, 0.5],
    [-2.0, 0.0, 0.5],
    [0.0, -2.0, 0.5],
], dtype=np.float32)


class CruiseMLP(nn.Module):
    """
    3 层 MLP：6 -> 64 -> 32 -> 2。
    为什么这么小：无 GPU 虚拟机训练，<10k 参数足够拟合专家 P 控制器；
    再大能过拟合专家本身，失去"神经网络学到扰动下鲁棒性"的意义。
    """

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(6, 64), nn.Tanh(),
            nn.Linear(64, 32), nn.Tanh(),
            nn.Linear(32, 2),
        )

    def forward(self, x):  # noqa: D401 - nn.Module 标准签名
        return self.net(x)


def _yaw_from_quat(q: np.ndarray) -> float:
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _expert_cmd(state: np.ndarray, waypoint: np.ndarray) -> np.ndarray:
    """
    脚本专家：输出世界坐标系下的期望速度 [vx, vy]。
    为什么是世界系而不是机体系：
      本模型用 qfrc_applied 施加世界系平动力（矢量推力 ROV 可任意方向平移），
      不需要先转航向再前进；把"指向航点的速度向量"分解成 vx/vy 直接下达即可。
    state = [x,y,z, yaw, vx,vy, wz]
    """
    dx = waypoint[0] - state[0]
    dy = waypoint[1] - state[1]
    dist = float(np.hypot(dx, dy))
    direction = np.arctan2(dy, dx)
    speed = float(np.clip(0.6 * dist, 0.0, 0.6))
    if dist < 0.25:
        speed = 0.0
    vx = speed * np.cos(direction)
    vy = speed * np.sin(direction)
    return np.array([vx, vy], dtype=np.float32)


def _make_sim(params: dict):
    """在进程内加载 MuJoCo 模型（离线用，与 ROS 节点同一份动力学）。"""
    common.set_mujoco_gl_backend()
    import mujoco  # noqa: WPS433 —— 必须在 set_mujoco_gl_backend 之后 import
    model = mujoco.MjModel.from_xml_path(common.find_model_path(params.get("model_path", "")))
    data = mujoco.MjData(model)
    dyn = common.WaterDynamics(params)
    return model, data, dyn


def _read_state(model, data) -> np.ndarray:
    pos = data.qpos[0:3].copy()
    yaw = _yaw_from_quat(data.qpos[3:7])
    vx, vy, vz = data.qvel[0:3]
    wx, wy, wz = data.qvel[3:6]
    return np.array([pos[0], pos[1], pos[2], yaw, vx, vy, wz], dtype=np.float32)


def collect_data(params: dict, steps: int = 4000):
    """
    离线采集：专家绕 4 个航点跑圈，记录 (特征, 专家指令)。
    为什么把专家指令一起记录：BC 的监督信号就是专家输出。
    """
    model, data, dyn = _make_sim(params)
    dt = model.opt.timestep
    feat_list, cmd_list = [], []
    wp_idx = 0
    for i in range(steps):
        t = i * dt
        state = _read_state(model, data)
        # 距离 < 0.3m 切下一个航点
        if np.hypot(EXPERT_WAYPOINTS[wp_idx][0] - state[0],
                    EXPERT_WAYPOINTS[wp_idx][1] - state[1]) < 0.3:
            wp_idx = (wp_idx + 1) % len(EXPERT_WAYPOINTS)
        wp = EXPERT_WAYPOINTS[wp_idx]
        expert = _expert_cmd(state, wp)
        feat = np.array([wp[0] - state[0], wp[1] - state[1],
                         (np.arctan2(wp[1] - state[1], wp[0] - state[0]) - state[3]),
                         state[4], state[5], state[6]], dtype=np.float32)
        feat_list.append(feat)
        cmd_list.append(expert)
        # 底层 P 推力（与 sim_node 同一套）
        twist = np.array([expert[0], expert[1], 0.0, 0.0, 0.0, 0.0])
        dyn.apply(data, t, model.body("rov").id, twist)
        mujoco_step(model, data)
    X = np.stack(feat_list)
    Y = np.stack(cmd_list)
    out = common.get_package_root() / "data"
    out.mkdir(exist_ok=True)
    np.savez(out / "cruise_dataset.npz", X=X, Y=Y)
    print(f"[collect] 采集样本 {X.shape[0]}，保存到 {out/'cruise_dataset.npz'}")
    return X, Y


def mujoco_step(model, data):
    import mujoco
    mujoco.mj_step(model, data)


def train(params: dict, epochs: int = 200, lr: float = 1e-3):
    """
    MLP 行为克隆：MSE 回归专家速度指令。
    为什么返回 train loss 曲线：PR 文档/性能评价需要"训练收敛曲线"。
    """
    root = common.get_package_root()
    ds_path = root / "data" / "cruise_dataset.npz"
    if not ds_path.exists():
        raise FileNotFoundError(
            f"找不到数据集 {ds_path}。请先运行: python3 main.py collect"
        )
    d = np.load(ds_path)
    X = torch.tensor(d["X"], dtype=torch.float32)
    Y = torch.tensor(d["Y"], dtype=torch.float32)
    net = CruiseMLP()
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    lossfn = nn.MSELoss()
    history = []
    for ep in range(epochs):
        opt.zero_grad()
        pred = net(X)
        loss = lossfn(pred, Y)
        loss.backward()
        opt.step()
        history.append(float(loss.item()))
        if ep % 40 == 0 or ep == epochs - 1:
            print(f"[train] epoch {ep:4d}/{epochs}  MSE={loss.item():.6f}")
    wdir = root / "weights"
    wdir.mkdir(exist_ok=True)
    wpath = common.find_weights_path(params.get("weights_path", ""))
    torch.save({"model": net.state_dict(), "history": history}, wpath)
    print(f"[train] 权重已保存: {wpath}")
    return history


def evaluate(params: dict, horizon: int = 12000, use_net: bool = True):
    """
    闭环评估：离线跑 horizon 步，MLP 或专家输出速度指令，统计航点跟踪误差。
    返回 dict：MAE(位置)、成功率(到过所有航点)、平均速度。
    为什么离线评估：ROS launch 跑通后，再用同一物理做量化指标，
    输出 MSE/MAE 满足作业"测试脚本打印量化指标"要求。
    与 collect_data 一致：按 wp_idx 顺序推进航点（到点切下一个），
    而不是每步选最近点——否则到达第一个点后会卡住不动。
    """
    root = common.get_package_root()
    wpath = common.find_weights_path(params.get("weights_path", ""))
    net = None
    if use_net:
        if not wpath.exists():
            raise FileNotFoundError(
                f"找不到权重 {wpath}。请先运行: python3 main.py train"
            )
        ckpt = torch.load(wpath, map_location="cpu")
        net = CruiseMLP()
        net.load_state_dict(ckpt["model"])
        net.eval()

    model, data, dyn = _make_sim(params)
    dt = model.opt.timestep
    err_hist, visited = [], set()
    wp_idx = 0
    for i in range(horizon):
        t = i * dt
        state = _read_state(model, data)
        wp = EXPERT_WAYPOINTS[wp_idx]
        # 距当前航点 < 0.4m 切下一个
        if np.hypot(wp[0] - state[0], wp[1] - state[1]) < 0.4:
            visited.add(wp_idx)
            wp_idx = (wp_idx + 1) % len(EXPERT_WAYPOINTS)
            wp = EXPERT_WAYPOINTS[wp_idx]
        # 跟踪误差 = 到当前目标航点的距离
        d_to_wp = float(np.hypot(wp[0] - state[0], wp[1] - state[1]))
        err_hist.append(d_to_wp)
        if use_net:
            feat = np.array([wp[0] - state[0], wp[1] - state[1],
                             (np.arctan2(wp[1] - state[1], wp[0] - state[0]) - state[3]),
                             state[4], state[5], state[6]], dtype=np.float32)
            with torch.no_grad():
                cmd = net(torch.tensor(feat).unsqueeze(0)).squeeze(0).numpy()
        else:
            cmd = _expert_cmd(state, wp)
        twist = np.array([cmd[0], cmd[1], 0.0, 0.0, 0.0, 0.0])
        dyn.apply(data, t, model.body("rov").id, twist)
        mujoco_step(model, data)
    mae = float(np.mean(err_hist))
    success = len(visited) == len(EXPERT_WAYPOINTS)
    result = {
        "use_net": use_net,
        "samples": horizon,
        "mae_position_m": round(mae, 4),
        "max_err_m": round(float(np.max(err_hist)), 4),
        "visited_waypoints": sorted(visited),
        "all_waypoints_hit": success,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result
