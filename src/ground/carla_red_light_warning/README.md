# CARLA 闯红灯预警（RLW）

在 CARLA 0.9.16 中实现**闯红灯预警**（Red Light Warning）完整闭环：
交通灯 API 感知（`get_traffic_light` + `get_stop_locations`）→ 判停距离
v²/(2a) → 三级滞回状态机（normal → approaching → violation_risk）→
跃迁事件。

与同仓库 AEB / BSD（LiDAR 感知）、CSL / LDW（地图路点）不同，本模块
数据源是**交通灯 API**——零传感器依赖；黄灯只提示接近，红灯且舒适
刹停不可行才判"将闯线"。

## 模块结构

```
carla_red_light_warning/
├── carla_red_light_warning/
│   ├── red_light_core.py   # 决策核心：判停距离/三级滞回/热身期（纯离线）
│   ├── bridge_node.py      # CARLA 桥接：交通灯感知 → /rlw/* 发布
│   ├── rlw_node.py         # 预警节点：状态机 → /rlw/state + /rlw/event
│   ├── carla_common.py     # CARLA 封装（复用 AEB/BSD/CSL/LDW 已验证实现）
│   └── demo.py             # 离线 / --live 真实 CARLA 一键演示
├── launch/main.launch.py   # 桥接+预警两节点一键启动
├── config/rlw_params.yaml  # 全部决策参数（顶层 /** 通配）
├── tests/                  # 38 个离线单元测试
└── main.py                 # doctor / test / build / demo 入口
```

## 快速开始

```bash
# 离线（无需 ROS 与 CARLA，目录：本模块根目录）
python3 main.py doctor
python3 main.py test        # 38/38 单元测试
python3 main.py demo        # 离线红灯接近场景，artifacts/ 出曲线 PNG + CSV

# CARLA 实测（先启动 CARLA 0.9.16，Python 需可导入 carla 包）
python3 main.py demo --live --host 192.168.1.100 --seconds 45

# ROS 2 launch（目录：ros2 工作空间根目录）
colcon build --packages-select carla_red_light_warning && source install/setup.bash
ros2 launch carla_red_light_warning main.launch.py host:=192.168.1.100
# 另开终端：ros2 topic echo /rlw/state
```

## 环境说明（carla Python 包）

live 演示需要 `import carla`。CARLA 0.9.16 官方 wheel 覆盖
Python 3.10–3.12（`PythonAPI/carla/dist/*.whl`），安装示例：

```bash
python3.12 -m pip install --user --break-system-packages \
    <CARLA_ROOT>/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl
python3.12 main.py demo --live --host <CARLA_IP> --seconds 45
```

宿主机默认 Python 为 3.13（无官方 wheel）时请用 python3.12 运行。

## 核心参数（config/rlw_params.yaml）

| 参数 | 默认 | 含义 |
|---|---|---|
| `a_comfort` | 2.5 | 舒适减速上限（m/s²） |
| `warn_enter` / `warn_exit` | 30 / 35 | approaching 进/出停止线距离（m，滞回） |
| `brake_margin` | 1.2 | violation 退出滞回系数（d > d_stop×margin 回退） |
| `warmup_ticks` | 2 | 热身拍数（冷启动灯态未稳定不输出决策） |

## 决策语义

黄灯只触发 approaching 提示（黄灯越线不违规）；红灯且
`dist < v²/(2·a_comfort)`（舒适刹停不可行）时触发 violation_risk。
事件仅跃迁拍发布一次：approach_start、violation_start、
violation_clear、approach_clear。

教程与实测结果见 `docs/ground/carla_red_light_warning.md`。
