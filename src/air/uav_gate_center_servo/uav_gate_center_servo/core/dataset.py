#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""门中心数据集：oracle 采集、按 episode 切分、水平镜像。"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

__all__ = ["Dataset", "collect_dataset", "split_by_episode", "mirror_batch"]


@dataclass
class Dataset:
    train_depth: np.ndarray
    train_label: np.ndarray
    val_depth: np.ndarray
    val_label: np.ndarray
    train_episode: np.ndarray
    val_episode: np.ndarray
    meta: Dict[str, object] = field(default_factory=dict)

    def __len__(self) -> int:
        return int(len(self.train_depth) + len(self.val_depth))

    def save(self, path: str) -> None:
        np.savez_compressed(
            path,
            train_depth=self.train_depth, train_label=self.train_label,
            val_depth=self.val_depth, val_label=self.val_label,
            train_episode=self.train_episode, val_episode=self.val_episode,
            meta_json=np.asarray(json.dumps(self.meta, ensure_ascii=False)),
        )

    @classmethod
    def load(cls, path: str) -> "Dataset":
        with np.load(path, allow_pickle=False) as d:
            meta = json.loads(str(d["meta_json"])) if "meta_json" in d.files else {}
            return cls(
                np.array(d["train_depth"], np.float32),
                np.array(d["train_label"], np.float32),
                np.array(d["val_depth"], np.float32),
                np.array(d["val_label"], np.float32),
                np.array(d["train_episode"], np.int32),
                np.array(d["val_episode"], np.int32),
                meta,
            )


def collect_dataset(env, episodes: int = 55, seed: int = 0,
                    max_steps: int = 300, verbose: bool = True) -> Dataset:
    """跑 oracle，每个 episode 必须干净通过；否则报错。"""
    depths: List[np.ndarray] = []
    labels: List[np.ndarray] = []
    ep_ids: List[np.ndarray] = []
    valid: List[float] = []
    for ep in range(int(episodes)):
        obs = env.reset(seed=int(seed) + ep)
        done = False
        info = {}
        steps = 0
        while not done and steps < max_steps:
            depths.append(obs["depth"].copy())
            labels.append(obs["label"].copy())
            ep_ids.append(np.full(1, ep, np.int32))
            valid.append(float(obs["valid_frac"]))
            obs, done, info = env.step(env.oracle_command())
            steps += 1
        if info.get("reason") != "pass":
            raise RuntimeError("oracle episode %d 未通过：%s" % (ep, info))
        if verbose:
            print("  episode %2d  steps=%3d  time=%.2fs" % (ep, steps, info["time"]))
    d = np.stack(depths).astype(np.float32)
    l = np.stack(labels).astype(np.float32)
    e = np.concatenate(ep_ids).astype(np.int32)
    if float(np.median(valid)) < 0.05:
        raise RuntimeError("深度图有效像素过低（median %.3f），检查渲染/headless"
                           % float(np.median(valid)))
    return split_by_episode(d, l, e, val_frac=0.2, seed=0,
                            meta={"episodes": int(episodes), "seed": int(seed),
                                  "samples": int(len(d))})


def split_by_episode(depth: np.ndarray, label: np.ndarray, episode: np.ndarray,
                     val_frac: float = 0.2, seed: int = 0,
                     meta: Dict[str, object] = None) -> Dataset:
    """按整段 episode 切分，避免相邻帧泄漏。"""
    ids = np.unique(episode)
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(ids))
    n_val = max(1, int(round(len(ids) * val_frac)))
    val_ids = set(ids[order[:n_val]].tolist())
    train_ids = set(ids[order[n_val:]].tolist())
    tr = np.array([int(e) in train_ids for e in episode], dtype=bool)
    va = ~tr
    m = dict(meta or {})
    m.update({"train_samples": int(tr.sum()), "val_samples": int(va.sum()),
              "train_episodes": len(train_ids), "val_episodes": len(val_ids)})
    return Dataset(depth[tr], label[tr], depth[va], label[va],
                   episode[tr].astype(np.int32), episode[va].astype(np.int32), m)


def mirror_batch(depth: np.ndarray, label: np.ndarray, mask: np.ndarray):
    """按 mask 对样本做水平镜像：深度左右翻转，只把 left 取反。"""
    d = np.array(depth, np.float32, copy=True)
    l = np.array(label, np.float32, copy=True)
    mask = np.asarray(mask, bool)
    if np.any(mask):
        d[mask] = d[mask][:, :, ::-1]
        l[mask, 1] *= -1.0
    return d, l
