"""通道名常量（不依赖 torch / mujoco，纯逻辑与归因模块共用）。

放在独立模块里，是为了让 ``attribute()`` 这类纯函数在没装 torch 的机器上也能 import，
而不是抛 ImportError。
"""

KEYS = ['px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']

# 检测器残差只覆盖前 6 个通道：位置差分与速度差分各 3 个
CHANNELS = KEYS[:6]
POSITION_GROUP = {'px', 'py', 'pz'}
VELOCITY_GROUP = {'vx', 'vy', 'vz'}
