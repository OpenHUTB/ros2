# 多车协同编队（Ground Vehicle Platoon）

地面载具多车协同编队仿真工程：Leader 沿参考轨迹匀速领航，3 辆 follower
沿同一条轨迹保持恒定弧长间距跟进，形成一字纵队（platoon），转弯处队形不散。

同一套编队控制律被两个平台复用（`platoon_core.py`）：

- **CARLA 版**（`carla_platoon.py`）：车辆状态取自 CARLA 真实位姿，控制量经
  `apply_control` 下发，需要在本地装 CARLA 的机器上运行。
- **2D 轻量版**（`sim_2d_platoon.py`）：matplotlib 渲染，可直接运行并生成
  效果动图（入库位置 `docs/img/ground/carla_platoon_demo.gif`）——即本编队算法
  的真实运行效果演示。

## 编队原理

- 一条参数化参考轨迹 `path`，按累计弧长 `s` 采样成密集点列。
- Leader 沿轨迹推进，第 `i` 辆 follower 的期望进度
  `s_i = s_leader - i * spacing`（沿轨迹滞后 leader 若干个车距），
  期望参考点 = 轨迹上弧长 `s_i` 处的点。
- 横向：纯跟踪（pure pursuit）追各自参考点。
- 纵向：进度误差 P 控制，使实际进度逼近期望进度，维持恒定间距。

## 文件结构

```
carla_platoon/
├── platoon_core.py       # 共享编队核心（运动学/纯跟踪/编队控制律）
├── carla_platoon.py      # CARLA 渲染版主脚本
├── sim_2d_platoon.py     # 2D 轻量仿真版主脚本
├── requirements.txt
└── README.md
```
> 效果动图统一入库 `docs/img/ground/carla_platoon_demo.gif`，源码目录不重复存放动图。

## 运行环境

2D 轻量版（生成动图）在本工程实测环境：**Ubuntu 22.04 + Python 3.12.11**
（3.12 为独立安装的 Python，非系统自带 3.10；2D 仿真为纯 numpy/matplotlib 实现，
不依赖系统编译器）。CARLA 渲染版需在装有对应版本 CARLA 的机器上运行。

## 运行

### 2D 版（本工程直接可跑，生成动图）

``` 
pip install numpy matplotlib Pillow
python3 sim_2d_platoon.py                      # 生成 platoon_demo.gif
python3 sim_2d_platoon.py --show               # 弹出窗口实时查看
python3 sim_2d_platoon.py --frames 300 --fps 30 --out platoon_demo.gif
```

### CARLA 版（需本机装 CARLA）

``` 
# 1) 启动 CARLA（如 /opt/carla/CarlaUE4.sh）
# 2) 让 Python 能找到 carla API（.egg 的 py<X.Y> 需与本地 Python 版本匹配）
export PYTHONPATH=/opt/carla/PythonAPI/carla/dist/carla-<ver>-py<X.Y>-linux-x86_64.egg:$PYTHONPATH
pip install numpy
python3 carla_platoon.py --map Town03 --n 4 --spacing 8 --v 6
```

## 参数说明

| 参数 | 含义 | 默认 |
|------|------|------|
| `n` | 车辆总数（含 leader） | 4 |
| `spacing` | 期望车距（沿轨迹弧长，m） | 8.0 |
| `v` | 巡航车速（m/s） | 6.0 |
| `lookahead` | 纯跟踪前视距离（m） | 6.0 |
| `L` | 轴距（m） | 2.8 |

## 测试

不依赖 CARLA / matplotlib 的核心测试（断言初始位姿不重合、稳态间距有界、leader 前视转向）：

``` 
python3 test_platoon.py
```

## 运行效果

动图 `docs/img/ground/carla_platoon_demo.gif` 为 2D 轻量版真实运行效果：
Leader 沿 S 形参考轨迹匀速领航，3 辆 follower 保持约 8 m 间距跟进，
连续转弯处队形保持稳定（实测稳态车距 7.7～8.0 m，与目标 8 m 一致）。

## 合规说明

本项目为课程/学习用途的仿真代码，不含真实道路行驶与真车控制，遵循开源许可。
