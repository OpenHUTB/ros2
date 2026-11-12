"""数据集构造：可降落性评分（教师蒸馏）与制导（专家行为克隆）。

两个数据集分开存：

- **评分数据集**：随机场景 × 随机位置扫出来的高度图，标签是 :func:`mapping.rule_landing_score`
  给出的规则评分。网络学的是"看一眼高度图就知道哪里能落"，部署时不再跑规则。
- **制导数据集**：在状态空间里随机采样（相对落点位置、速度、推力权限、高度），
  标签是 :func:`landing.expert_velocity` 的专家速度指令。这一步不需要仿真，
  因为专家律本身就是闭式的；采样范围覆盖 1~8 m 高度、0.5~1.0 的推力权限。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from . import landing as L
from . import mapping as M

#: 建图用的场景（有建筑物/树/墙，才有"哪里不能落"的区分度）
MAP_SCENES = ("open_ground", "corridor", "obstacle_field", "urban_block")
#: 每种场景模型复用的轮数（make_scene 要编译模型，反复建太慢）
MAPS_PER_MODEL = 6


@dataclass
class ScanConfig:
    """建图时的采样设置。"""

    scans: int = 8
    altitude_range: tuple = (1.5, 5.0)
    yaw_step_deg: float = 45.0
    move_radius: float = 1.5


# --------------------------------------------------------------------------- #
# 评分数据集
# --------------------------------------------------------------------------- #
def scan_scene(api, scene_name: str, seed: int, rng: np.random.Generator,
               cfg: Optional[ScanConfig] = None) -> M.HeightMap:
    """在场景里换几个位置/朝向扫一遍，把点云并成一张高度图。"""
    cfg = cfg or ScanConfig()
    import mujoco

    quad = api.Quadrotor(scene=api.make_scene(scene_name, seed=seed), dt=0.01, substeps=4)
    mapper = M.DepthMapper(api).bind(quad)
    height_map = M.HeightMap(M.GridConfig())

    alt = float(rng.uniform(*cfg.altitude_range))
    x0 = float(rng.uniform(-4.0, 4.0))
    y0 = float(rng.uniform(-4.0, 4.0))
    for k in range(cfg.scans):
        ang = 2 * np.pi * k / max(cfg.scans, 1)
        radius = cfg.move_radius * (1.0 if k % 2 else 0.0)
        pos = (x0 + radius * np.cos(ang), y0 + radius * np.sin(ang), alt)
        quad.reset(pos=pos, yaw=np.deg2rad(cfg.yaw_step_deg) * k)
        mujoco.mj_forward(quad.model, quad.data)
        height_map.update(mapper.scan(quad.data))
    return height_map


def build_score_dataset(api, n_maps: int = 120, seed: int = 0,
                        cfg: Optional[ScanConfig] = None,
                        progress=None) -> Dict[str, Any]:
    """生成评分数据集：``X`` 为 (N,1,R,C) 归一化高度图，``Y`` 为同尺寸规则评分。"""
    cfg = cfg or ScanConfig()
    rng = np.random.default_rng(seed)
    grids, scores, scenes, seeds = [], [], [], []
    for scene in MAP_SCENES:
        for model_seed in range(max(1, n_maps // (len(MAP_SCENES) * MAPS_PER_MODEL))):
            for _ in range(MAPS_PER_MODEL):
                hm = scan_scene(api, scene, model_seed, rng, cfg)
                if not np.any(hm.observed):
                    continue
                xy = rng.uniform(-4.0, 4.0, size=2)
                grids.append(M.height_map_input(hm))
                scores.append(M.rule_landing_score(hm, position_xy=xy)[None, :, :])
                scenes.append(scene)
                seeds.append(model_seed)
                if progress is not None and len(grids) % 20 == 0:
                    progress(len(grids))
    if not grids:
        raise RuntimeError("没有采到任何地图，检查场景与雷达配置")
    return {
        "X": np.stack(grids).astype(np.float32),
        "Y": np.stack(scores).astype(np.float32),
        "scene": np.array(scenes),
        "seed": np.array(seeds, dtype=np.int32),
        "grid": np.array(M.GridConfig().shape, dtype=np.int32),
    }


# --------------------------------------------------------------------------- #
# 制导数据集
# --------------------------------------------------------------------------- #
def build_guidance_dataset(n_samples: int = 4000, seed: int = 0) -> Dict[str, Any]:
    """随机采样状态并打上专家速度指令标签。"""
    rng = np.random.default_rng(seed)
    cfg = L.LandingConfig()
    feats = np.empty((n_samples, L.GUIDANCE_DIM), dtype=np.float32)
    labels = np.empty((n_samples, 3), dtype=np.float32)
    gain_sets = [
        np.ones(4),
        np.array([0.0, 1.0, 1.0, 1.0]),
        np.array([1.0, 0.0, 1.0, 1.0]),
        np.array([0.5, 1.0, 1.0, 1.0]),
        np.array([1.0, 0.7, 0.9, 1.0]),
        np.array([0.0, 0.0, 1.0, 1.0]),
        np.array([0.0, 1.0, 0.0, 1.0]),
    ]
    for i in range(n_samples):
        altitude = float(rng.uniform(0.2, 8.0))
        rel_xy = rng.uniform(-7.0, 7.0, size=2)
        rel = np.array([rel_xy[0], rel_xy[1], -altitude], dtype=float)
        vel = rng.uniform(-2.5, 2.5, size=3)
        gains = np.asarray(gain_sets[int(rng.integers(len(gain_sets)))], dtype=float)
        if rng.random() < 0.2:  # 两桨失效的极端权限
            gains = gains.copy()
            gains[int(rng.integers(4))] = 0.0
        feats[i] = L.guidance_features(rel, vel, gains, altitude)
        labels[i] = L.expert_velocity(rel, vel, gains, altitude, cfg)
    return {"X": feats, "Y": labels}


# --------------------------------------------------------------------------- #
# 存取与划分
# --------------------------------------------------------------------------- #
def save_dataset(path: str | Path, data: Dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    meta = {k: v for k, v in data.items() if k not in ("X", "Y")}
    np.savez_compressed(path, X=data["X"], Y=data["Y"],
                        meta=np.array(json_dumps(meta)))
    return path


def load_dataset(path: str | Path) -> Dict[str, Any]:
    with np.load(str(path), allow_pickle=False) as z:
        out: Dict[str, Any] = {"X": z["X"], "Y": z["Y"]}
        for key, value in json_loads(str(z["meta"])).items():
            out[key] = value
    return out


def json_dumps(obj: Dict[str, Any]) -> str:
    import json

    def default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (np.integer, np.floating)):
            return value.item()
        return value

    return json.dumps(obj, default=default)


def json_loads(text: str) -> Dict[str, Any]:
    import json

    return json.loads(text)


def split_samples(n: int, fractions=(0.8, 0.1, 0.1), seed: int = 0) -> Dict[str, np.ndarray]:
    """按样本随机划分（评分/制导数据集都是独立样本，不存在时序泄漏）。"""
    rng = np.random.default_rng(seed)
    idx = rng.permutation(n)
    n_train = int(round(fractions[0] * n))
    n_val = int(round(fractions[1] * n))
    out = {
        "train": idx[:n_train],
        "val": idx[n_train:n_train + n_val],
        "test": idx[n_train + n_val:],
    }
    return out
