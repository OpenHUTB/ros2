# mujoco_rov_depth_control — 水下 ROV 定深控制实验

## 1. 概述

用 **MuJoCo** 物理引擎搭一个近中性浮力的 ROV 刚体模型（含线性/二次水阻尼、垂向推进器），
实现串级定深控制器：依次定深 −5 m → −2 m → −0.5 m，每段量化稳态误差与超调。
水下机器人（潜水器）方向的控制链路由此完整跑通，且不依赖任何外部世界包。

## 2. 与已有示例的区别

| 对比项 | 官方/常见示例 | 本模块 |
|---|---|---|
| 物理 | 直接在引擎里"推箱子" | 显式建模净浮力 + 线性/二次阻尼（MuJoCo 本身不含浮力与流体阻力） |
| 控制 | 单环 PD 打满推力 | 串级：位置误差 → 期望垂速（限速）→ 期望加速度 → 推力限幅 |
| 验收 | 只看曲线"像不像" | 三段定深逐段算稳态误差，输出 PASS/FAIL |
| 依赖 | HoloOcean 需数 GB 世界包 | 纯 MuJoCo（OpenHUTB 自带 `mujoco/` 引擎），零下载 |

## 3. 环境要求

- Windows / Linux 均可；Python 3.10
- `mujoco`（3.17+）、`numpy`、`matplotlib`、`PyYAML`
- 不需要 CARLA 服务端，不需要 GPU

> 说明：HoloOcean 路线需要另外下载数 GB 的世界包和独立渲染器，本机没有对应资源；
> OpenHUTB 的水下方向本身就是基于 MuJoCo 插件，因此这里用 MuJoCo 复现控制链路。

## 4. 安装

```bat
python -m pip install mujoco numpy matplotlib pyyaml
```

## 5. 运行

```bat
cd src\water\mujoco_rov_depth_control
python main.py            :: 或 main.bat / bash main.sh
```

## 6. 预期输出

```text
[rov] 目标深度序列: [-5.0, -5.0, -2.0, -2.0, -0.5, -0.5]，每个保持 7.0 s，仿真步长 0.002 s
[rov] 实际机体质量 16.60 kg，净浮力 3.3 N
[rov] t=0.0s 切换到目标深度 -5.00 m
[rov] t=14.0s 切换到目标深度 -2.00 m
[rov] t=28.0s 切换到目标深度 -0.50 m
[rov] 定深 0：目标 -5.00 m，稳态平均误差 0.013 m，最大 0.042 m
[rov] 定深 1：目标 -2.00 m，稳态平均误差 0.002 m，最大 0.005 m
[rov] 定深 2：目标 -0.50 m，稳态平均误差 0.000 m，最大 0.001 m
[rov] 结论：PASS（三段定深均收敛）
```

## 7. 输出文件

| 文件 | 内容 |
|---|---|
| `docs/result.png` | 深度跟踪曲线（含设定值）+ 垂向推力曲线 |
| `docs/depth_log.csv` | 50 Hz 逐点：`t / setpoint / depth / error / thrust / vz` |
| `docs/summary.txt` | 质量、净浮力、控制律参数、各段稳态误差 |

## 8. 目录结构

```text
mujoco_rov_depth_control/
├── main.py              # 模型内联在 rov_xml() 里，无需额外 XML 文件
├── main.sh / main.bat
├── config/params.yaml   # 质量/尺寸/阻尼/推力限幅/控制增益/任务序列
├── docs/
└── README.md
```

## 9. 已知限制

1. **浮力与阻尼是显式建模的**，不是引擎自带：`xfrc_applied` 里补浮力 + 阻尼，
   同时 MuJoCo 自己会施加重力，所以补浮力时不能再减一次重量（这是本次调试的主要坑）。
2. 单自由度（垂向）控制，没有做横滚/俯仰姿态控制。
3. 模型是简化刚体，不与 OpenHUTB 的 UE4 场景做视觉联调。
