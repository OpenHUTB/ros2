"""射线传感器的测距噪声与丢束（纯逻辑，不依赖 mujoco）。

单独成模块的原因：这组逻辑只有 numpy，不需要 MuJoCo 就能测；放在 sensors.py 里会
连带 `import mujoco`，没装 MuJoCo 的机器上测试只能全 skip。

顺序约定（评审指出过的坑）：
1. 先判定哪些束丢束；
2. **只对有效回波加噪**；
3. 最后把丢束项精确写回量程上限，保证「随机无回波」的语义不被噪声污染。
"""
import numpy as np


def apply_sensor_noise(ranges, sigma, dropout, rng, limit):
    """返回加噪后的射线距离数组（float32）。

    ranges: 无噪声的射线距离；sigma: 高斯噪声标准差(m)；dropout: 单束丢失概率；
    rng: numpy Generator（同一种子可复现）；limit: 量程上限。
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    if sigma <= 0.0 and dropout <= 0.0:
        return ranges
    n = ranges.shape[0]
    limit = float(limit)
    dropped = rng.random(n) < dropout if dropout > 0.0 else np.zeros(n, dtype=bool)
    out = np.array(ranges, dtype=np.float32, copy=True)
    if sigma > 0.0:
        noise = rng.normal(0.0, sigma, n).astype(np.float32)
        valid = ~dropped
        out[valid] = np.clip(out[valid] + noise[valid], 0.0, limit)
    if dropout > 0.0:
        out[dropped] = limit
    return out.astype(np.float32)
