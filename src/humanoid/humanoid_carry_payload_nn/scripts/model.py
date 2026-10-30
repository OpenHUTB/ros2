#!/usr/bin/env python3
"""model.py —— 平衡控制器：PD 专家 与 神经网络残差策略。

两级结构（"基座步态 + 神经残差"）：
  1. ``PDExpert``：经典 PD 姿态反馈，用躯干 roll/pitch 及其角速度直接生成
     踝/髋关节的补偿角，并叠加与负载质量成正比的俯仰前馈项（offset）。
     它是采集示教数据的"专家"，也是评测时的对照基线；
  2. ``BalanceMLP``：纯 numpy 实现的 3 层全连接网络（tanh 激活），输入为
     躯干姿态、角速度、负载质量、速度/转向指令与步态相位，输出 3 个残差
     补偿角（踝俯仰 / 踝横滚 / 髋俯仰）。权重训练方式为行为克隆
     （见 train_mlp.py）：拟合专家在 0/2/4 kg 负载下的补偿输出。

之所以只让网络输出"残差"，是因为步态本身由几何规划给出（gait.py），
网络只负责搬运负载带来的姿态偏差补偿，避免网络同时学会走路与平衡导致
训练目标和评价口径不清。
"""

import json
import os

import numpy as np

#: 网络输入特征（顺序固定，训练与推理必须一致）
FEATURE_NAMES = [
    "roll",           # 躯干横滚角 (rad)
    "pitch",          # 躯干俯仰角 (rad)
    "roll_rate",      # 横滚角速度 (rad/s)
    "pitch_rate",     # 俯仰角速度 (rad/s)
    "payload_norm",   # 负载质量归一化 (m / 4kg)
    "v_cmd",          # 前进速度指令 (m/s)
    "w_cmd",          # 转向指令 (rad/s)
    "phase_sin",      # 步态相位正弦
    "phase_cos",      # 步态相位余弦
]

#: 网络输出残差（对称施加到左右腿）
OUTPUT_NAMES = ["ankle_pitch", "ankle_roll", "hip_pitch"]


class PDExpert:
    """PD 姿态反馈专家（带负载质量前馈）。"""

    def __init__(self, kp_pitch=0.5, kd_pitch=0.2, kp_roll=0.0, kd_roll=0.0,
                 k_mass_hip=0.05, k_hip=-0.3, pitch_ref=0.0, corr_limit=0.50):
        self.kp_pitch = float(kp_pitch)
        self.kd_pitch = float(kd_pitch)
        self.kp_roll = float(kp_roll)
        self.kd_roll = float(kd_roll)
        self.k_mass_hip = float(k_mass_hip)
        self.k_hip = float(k_hip)
        self.pitch_ref = float(pitch_ref)
        self.corr_limit = float(corr_limit)

    @staticmethod
    def feature_dict(roll, pitch, roll_rate, pitch_rate, payload_mass, v_cmd, w_cmd, phase):
        return {
            "roll": roll, "pitch": pitch,
            "roll_rate": roll_rate, "pitch_rate": pitch_rate,
            "payload_norm": payload_mass / 4.0,
            "v_cmd": v_cmd, "w_cmd": w_cmd,
            "phase_sin": np.sin(2 * np.pi * phase),
            "phase_cos": np.cos(2 * np.pi * phase),
        }

    def act(self, features):
        """返回补偿角字典（rad）。"""
        err = features["pitch"] - self.pitch_ref
        ankle_pitch = self.kp_pitch * err + self.kd_pitch * features["pitch_rate"]
        ankle_roll = self.kp_roll * features["roll"] + self.kd_roll * features["roll_rate"]
        # 负载前馈：重物挂在胸前使重心前移，用髋关节让躯干后仰、把重心拉回支撑面
        hip_pitch = self.k_hip * err + self.k_mass_hip * features["payload_norm"]
        out = {"ankle_pitch": ankle_pitch, "ankle_roll": ankle_roll, "hip_pitch": hip_pitch}
        return {k: float(np.clip(v, -self.corr_limit, self.corr_limit)) for k, v in out.items()}


class BalanceMLP:
    """3 层全连接网络（numpy 前向推理），权重文件为 .npz。"""

    def __init__(self, weights=None, hidden=(32, 32), corr_limit=0.45, mean=None, std=None):
        self.corr_limit = float(corr_limit)
        self.hidden = hidden
        self.mean = np.zeros(len(FEATURE_NAMES)) if mean is None else np.asarray(mean, dtype=float)
        self.std = np.ones(len(FEATURE_NAMES)) if std is None else np.asarray(std, dtype=float)
        if weights is None:
            rng = np.random.default_rng(0)
            sizes = [len(FEATURE_NAMES)] + list(hidden) + [len(OUTPUT_NAMES)]
            self.weights = []
            for i in range(len(sizes) - 1):
                self.weights.append(rng.normal(0.0, 0.05, (sizes[i], sizes[i + 1])))
                self.weights.append(np.zeros(sizes[i + 1]))
        else:
            self.weights = list(weights)

    # ------------------------------------------------------------------
    def features(self, roll, pitch, roll_rate, pitch_rate, payload_mass, v_cmd, w_cmd, phase):
        raw = np.array([
            roll, pitch, roll_rate, pitch_rate, payload_mass / 4.0, v_cmd, w_cmd,
            np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase),
        ], dtype=float)
        return raw

    def forward_normalized(self, x):
        h = (np.asarray(x, dtype=float) - self.mean) / self.std
        for i in range(0, len(self.weights), 2):
            h = h @ self.weights[i] + self.weights[i + 1]
            if i + 2 < len(self.weights):
                h = np.tanh(h)
        return h

    def act(self, features):
        x = np.array([features[name] for name in FEATURE_NAMES], dtype=float)
        y = self.forward_normalized(x)
        return {name: float(np.clip(y[i], -self.corr_limit, self.corr_limit))
                for i, name in enumerate(OUTPUT_NAMES)}

    # ------------------------------------------------------------------
    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        payload = {f"w{i}": w for i, w in enumerate(self.weights)}
        payload["mean"] = self.mean
        payload["std"] = self.std
        np.savez(path, **payload)
        meta = {"features": FEATURE_NAMES, "outputs": OUTPUT_NAMES,
                "hidden": list(self.hidden), "corr_limit": self.corr_limit}
        with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path, corr_limit=None):
        data = np.load(path)
        n_layers = len([k for k in data.files if k.startswith("w")])
        weights = [data[f"w{i}"] for i in range(n_layers)]
        hidden = tuple(weights[0].shape[1:]) + tuple(weights[-1].shape[:-1])
        return cls(weights=weights, hidden=hidden,
                   corr_limit=corr_limit or 0.45,
                   mean=data["mean"], std=data["std"])
