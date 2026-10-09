"""任务2-A：基于深度相机 + 神经网络的前向避障。

原理：
1. 从前视深度相机取归一化深度图 d ∈ [0,1]（对应 0~80 m）；
2. 把图像宽度等分成 B=16 个水平条带，每条约取中值距离，得到深度特征向量
   ``x = [d_1, ..., d_B]``（越近值越小）；
3. 输入 ``DepthAvoidNet``（2 层 MLP），输出 (横向修正, 纵向修正) ∈ [-1,1]；
4. 叠加到基础前向速度上：
       vx = v_fwd * (1 + k_long * a_long)
       vy = k_lat * a_lat
   并在最近距离 < ``safe_dist`` 时强制刹车/悬停，作为网络之外的安全兜底。
"""

import numpy as np

from common.models import DepthAvoidNet

try:  # pragma: no cover
    import torch
    _HAS_TORCH = True
except ImportError:  # pragma: no cover
    torch = None
    _HAS_TORCH = False


class DepthAvoider:
    def __init__(self, client, bins=16, v_fwd=1.5, safe_dist=1.2,
                 k_lat=1.2, k_long=0.5):
        self.client = client
        self.bins = bins
        self.v_fwd = v_fwd
        self.safe_dist = safe_dist
        self.k_lat = k_lat
        self.k_long = k_long
        self.net = DepthAvoidNet(bins)
        if _HAS_TORCH:
            self.net.eval()

    # ------------------------------------------------------------ 特征提取
    def _depth_bins(self, depth_norm):
        """depth_norm: HxW ∈ [0,1]。返回每列条带的中值真实距离（米）。"""
        d = depth_norm * 80.0
        h, w = d.shape
        edges = np.linspace(0, w, self.bins + 1, dtype=int)
        feat = np.zeros(self.bins, dtype=np.float32)
        for i in range(self.bins):
            col = d[:, edges[i]:edges[i + 1]]
            feat[i] = np.median(col) / 80.0  # 归一化
        return feat

    # ------------------------------------------------------------ 推理
    def infer(self, feat):
        if not _HAS_TORCH:
            # 无 torch 时退化到手工规则：近的一侧让开
            near_idx = int(np.argmin(feat))
            a_lat = -1.0 if near_idx < self.bins // 2 else 1.0
            a_long = 0.0
            return np.array([a_lat, a_long], dtype=np.float32)
        with torch.no_grad():
            t = torch.from_numpy(feat).unsqueeze(0)
            return self.net(t).squeeze(0).cpu().numpy()

    # ------------------------------------------------------------ 控制
    def on_tick(self, client):
        depth = client.get_depth("0")
        feat = self._depth_bins(depth)
        min_d = float(np.min(feat)) * 80.0
        if min_d < self.safe_dist:
            client.move_by_velocity(0.0, 0.0, 0.0)   # 安全悬停
            return
        a_lat, a_long = self.infer(feat)
        vx = self.v_fwd * (1.0 + self.k_long * a_long)
        vy = self.k_lat * a_lat
        client.move_by_velocity(vx, vy, 0.0)

    def run(self, seconds=30, rate_hz=10.0):
        self.client.start()
        self.client.spin(seconds, on_tick=self.on_tick, rate_hz=rate_hz)
        self.client.destroy()
