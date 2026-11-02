"""CoM 反馈平衡基元（analytic balance primitive）。

为什么需要它
------------
G1 的踝关节 kp 只有 40 Nm/rad，而倒立摆的失稳刚度是

    m·g·h ≈ 35.11 × 9.81 × 0.80 ≈ 275 Nm/rad

踝关节最大力矩只有 ±50 Nm（刚好够撑住约 10° 倾角）。也就是说，**只给固定目标角
的 PD 控制，在数学上不可能让机器人站住**——实测正是如此：pitch 从 2° 单调漂到
50°，CoM_x 从 0.011 m 漂到 0.595 m，1.2 s 内前倾倒地，而且任何增益组合都救不
回来（踝 kp 从 40 扫到 320，存活时间反而从 1.86 s 降到 1.02 s，因为力矩已饱和）。

加上质心位置/速度反馈后（Kc=10, Kv=1）可以稳定站立 10 s 以上（测试跑满）。

本模块把这段解析反馈独立出来，作为神经网络的**稳定基元**：NN 只输出残差偏置，
负责学迈步与转向；基元负责不摔倒。这样回合长度从 1.2 s 提升到 10 s+，
PPO 才有足够时间探索步态。

符号标定（实测）
----------------
``ankle_pitch`` 正向 = 身体后仰。因此对前倾（CoM 偏前）必须给**正**的修正量；
写成 ``-k*com_err`` 会变成正反馈，越修越倒。

输入约定
--------
``com_err`` 是**相对双脚中心**的前向偏差（平移不变），由
:meth:`~.sim_core.G1Sim.com_error` 提供。不要传世界系绝对坐标 ``com_x``——
那样机器人一旦往前走，误差就会线性增长，平衡器会去对抗前进。

实测性能
--------
- 站立：> 10 s（Kc ∈ [5, 10]，Kv ∈ [0.5, 2]）
- 抗初始关节角噪声：±0.05 rad 时 5/5 站住，±0.10 rad 时 4/5
- 控制频率：必须 ≥ 50 Hz（25 Hz 仅 0.36 s）
- 抗外力冲击较弱（0.5 m/s 即倒），这正是需要 NN 学迈步的原因
"""

from __future__ import annotations

import numpy as np

# 实测能稳定站立的增益区间：Kc ∈ [5, 10]，Kv ∈ [0.5, 2]
DEFAULT_KC = 10.0
DEFAULT_KV = 1.0

# 修正量在踝与髋之间的分配比例（0.5 表示髋承担一半）
DEFAULT_HIP_SHARE = 0.5


class ComBalance:
    """质心反馈平衡基元。

    修正量 ``c = Kc·com_err + Kv·(com_vx − v_des)``，
    同时加到 ``ankle_pitch`` 与 ``hip_pitch``（按 ``hip_share`` 分配）。

    Parameters
    ----------
    kc, kv:
        位置/速度反馈增益。超出 [5, 10] / [0.5, 2] 容易振荡或站不住。
    hip_share:
        髋关节承担的修正比例。
    q_min, q_max:
        关节角限位（长度 12，顺序同 ``LEG_JOINTS``）。为 ``None`` 时不裁剪。
    """

    # 12 个腿部关节中 hip_pitch / ankle_pitch 的下标
    IDX_HIP_PITCH = (0, 6)
    IDX_ANKLE_PITCH = (4, 10)

    def __init__(
        self,
        kc: float = DEFAULT_KC,
        kv: float = DEFAULT_KV,
        hip_share: float = DEFAULT_HIP_SHARE,
        q_min: np.ndarray | None = None,
        q_max: np.ndarray | None = None,
    ):
        self.kc = float(kc)
        self.kv = float(kv)
        self.hip_share = float(hip_share)
        self.q_min = None if q_min is None else np.asarray(q_min, dtype=np.float64)
        self.q_max = None if q_max is None else np.asarray(q_max, dtype=np.float64)

    def correction(self, com_err: float, com_vx: float, v_des: float = 0.0) -> float:
        """返回标量修正量（弧度），正值使身体后仰。"""
        return self.kc * float(com_err) + self.kv * (float(com_vx) - float(v_des))

    def targets(
        self,
        nominal: np.ndarray,
        com_err: float,
        com_vx: float,
        v_des: float = 0.0,
    ) -> np.ndarray:
        """在标称姿态上叠加平衡修正，返回 12 维关节目标角。"""
        g = np.array(nominal, dtype=np.float64, copy=True)
        c = self.correction(com_err, com_vx, v_des)
        for i in self.IDX_ANKLE_PITCH:
            g[i] += c
        for i in self.IDX_HIP_PITCH:
            g[i] += self.hip_share * c
        if self.q_min is not None and self.q_max is not None:
            g = np.clip(g, self.q_min, self.q_max)
        return g

    def __repr__(self) -> str:
        return f"ComBalance(kc={self.kc}, kv={self.kv}, hip_share={self.hip_share})"
