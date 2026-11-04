"""把检测器的 6 维残差按通道分摊，供在线诊断指出「大概哪个观测通道不对」。

两个检测器（GRU 预测残差、跨通道一致性残差）都产出 6 维残差：
位置差分速度与两端速度均值之差（px, py, pz），以及速度差分加速度与两端加速度均值之差
（vx, vy, vz）。残差除以训练集尺度后取绝对值，归一化即得到各通道占比。
这只是相关性归因，不是因果解释，也不能区分「传感器坏了」和「观测层注入了扰动」。
"""
import numpy as np

from train_evaluate import KEYS

CHANNELS = KEYS[:6]
DOMINANT_SHARE = 0.2
TOP_CHANNELS = 2


def attribute(residual, scale, threshold=DOMINANT_SHARE):
    """返回 {'channels': {通道名: 占比}, 'dominant': [主导通道...]}。"""
    residual = np.asarray(residual, dtype=float)
    scale = np.maximum(np.asarray(scale, dtype=float), 1e-12)
    normalized = np.abs(residual / scale)
    total = float(normalized.sum())
    if total <= 0:
        return {'channels': {name: 0.0 for name in CHANNELS}, 'dominant': []}
    share = normalized / total
    order = np.argsort(share)[::-1]
    dominant = [CHANNELS[i] for i in order if share[i] >= threshold][:TOP_CHANNELS]
    return {'channels': {name: round(float(share[i]), 3) for i, name in enumerate(CHANNELS)},
            'dominant': dominant}
