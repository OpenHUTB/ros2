# winter 场景无人机圆周飞行

本实验把 MAVROS 绕圈教程的控制思路适配到 Unreal Engine 4.27 的 winter/AirSim 场景，使用 Python RPC 实现起飞、定高、圆周跟踪与悬停。本示例直接连接 AirSim，不需要 ROS master 或 MAVROS。

## 环境与代码

- 已有 winter 场景、AirSim 插件，多旋翼 Drone1。
- Python 环境能够执行 `python -c "import airsim"`。
- 源码：仓库 `src/air/winter_circle/winter_circle.py`。
- 配置参考：`src/air/winter_circle/config/settings.example.json`，不要直接覆盖已有 AirSim 配置。

## 启动

在 UE4.27 中打开 winter.uproject，选择场景并点击 Play。默认地址 127.0.0.1:41451；不要同时启动 Blocks 占用同一端口。无人机应已着地，并停止其他控制器。

从仓库根目录运行：

```bash
# 仅显示参数，不连接、不起飞
python src/air/winter_circle/winter_circle.py
# 实际运行，完成后悬停
python src/air/winter_circle/winter_circle.py --fly --radius 2 --height 3 --speed 0.5 --laps 1
```

添加 `--land` 在正常完成后降落；添加 `--clockwise` 改为从上方看顺时针。Ctrl+C 会尝试悬停。虚拟机远程连接时添加 `--host <Windows主机IP>`，以实际网络为准。

## 参数与轨迹

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| --vehicle | Drone1 | settings 中飞行器名 |
| --radius | 2 | 圆半径，米 |
| --height | 3 | 相对初始位置升高距离，米 |
| --speed | 0.5 | 水平合速度上限，米/秒 |
| --laps | 1 | 目标圈数 |
| --port | 41451 | AirSim RPC 端口 |

圆心为初始位置向北偏移一个半径，因此起始位置就在圆周上。前三秒逐渐增加切向速度，累计实际极角判断圈数。yaw 固定为零，绕圈不等于机头持续转向。

## 接口和坐标

| MAVROS 教程接口 | AirSim 适配 |
| --- | --- |
| local_position/pose | getMultirotorState |
| OFFBOARD、解锁 | enableApiControl、armDisarm |
| 起飞与高度控制 | takeoffAsync、moveToZAsync |
| setpoint_velocity/cmd_vel | moveByVelocityZAsync |

程序在 NED 世界系计算：x 北、y 东、z 向下。目标 z=初始 z-height；ENU→NED 的通用向量变换为 (east,north,up)→(north,east,-up)。全程在 NED 中计算时无需重复转换。

设距离圆心为 d，径向单位向量为 u，径向修正为 0.5(R-d)u，切向速度为 omega*R*(-u_y,u_x)，两者相加后限幅。高度由 AirSim 定高速度接口跟踪。

## 碰撞与运行边界

历史接地信息可能保存在碰撞记录中。脚本记录初始时间戳，只对本次运行的新碰撞停止任务，并输出碰撞物体。若出现 `New collision`，检查物体名称和起飞空间，不要关闭碰撞保护。

定高误差、路径越界、任务超时、控制循环延迟也会终止绕圈并尝试悬停。通信断开不能保证悬停指令送达。没有自主避障和地形跟随，height 不是沿途离地高度，winter 坡地应选空旷位置。

## 验证记录与运行效果

已检查 Python 语法、无飞行参数预览、坐标转换、合速度限幅、历史/新增碰撞分支。简化平面运动积分完成一圈，最大半径误差约 0.0063 米；该结果属于数学离线检查，不代表 AirSim 实测误差。

用户提供的终端截图显示：半径 2 米、高度差 3 米、速度 0.5 米/秒、圈数 1，已连接 AirSim，旧 Landscape_7 碰撞记录被识别为历史事件，最终输出 Circle complete。该记录支持脚本达到圈数完成条件；没有轨迹日志，无法量化真实半径和高度误差。

实测时记录命令、圈数、运行日志，并附完整绕圈视频或连续截图；不要将单张场景图片作为绕圈完成证据。

## 来源

参考用户提供 tutorials.tar 中 tutorial_basic/src/circle_node.cpp 的控制思路。Python 适配由 OpenAI Codex 辅助编写，原始源码及许可归属以原材料为准。

## 用户实测材料（2026-11-02）

![winter 场景录屏第 10 秒截图](../img/air/winter_circle/winter_flight.png)

上图取自用户录屏第 10 秒，可见 winter 场景中的无人机。画面出现 entering hover mode for safety 提示；单帧不能证明完整圆形轨迹或定位精度。

![脚本完成一圈的终端输出](../img/air/winter_circle/circle_complete.png)

终端记录包含实际命令与 Circle complete。录屏与终端截图分别作为运行画面和程序完成记录，不作为定量轨迹评估。

<video controls preload="metadata" width="100%">
  <source src="../../img/air/winter_circle/winter_circle_demo.mp4" type="video/mp4">
  浏览器不支持视频播放，请下载视频查看。
</video>

[下载原始运行录屏](../img/air/winter_circle/winter_circle_demo.mp4)

## PR #850 复核后的改进

本节记录代码修改，不代表新一轮在线飞行已通过。原视频与 `Circle complete` 记录属于旧版实测材料。

### 客户端安装

场景仍需自行准备 UE4.27 winter 工程及兼容的 AirSim 服务端；仓库不分发该场景。这里使用 PyPI `airsim==1.8.1` 客户端，不是 CARLA Python API。依赖清单在 `src/air/winter_circle/requirements.txt`。审核者使用过 Windows 11 / Python 3.10.21；以下固定版本组合尚未在全新环境验证。

建议单独创建 Python 3.10 环境。在 Windows CMD、仓库根目录执行：

```bat
py -3.10 -m venv .venv-winter
.venv-winter\Scripts\activate
python -m pip install setuptools wheel numpy==1.24.4 msgpack-rpc-python==0.4.1
python -m pip install --no-build-isolation -r src/air/winter_circle/requirements.txt
python -c "import airsim; print(airsim.__file__)"
```

分步安装是因为 AirSim 旧版打包脚本会在构建时导入 NumPy 和 RPC 依赖。若已有来自官方 PythonClient 的 `airsim`，请用独立环境避免混装，并核对上面的导入路径。

### 指令结果和控制周期

连接失败会显示主机、端口和启动检查提示。起飞、爬升、速度、悬停、降落均调用 future 的 `.get()`，捕获 RPC 异常，并将非 True 返回值作为拒绝执行处理。速度指令改为 0.1 秒有限段，上一段确认完成才发送下一段，不积累未检查的 future。

控制循环按单调时钟计算 0.1 秒目标截止时间，提前完成才等待；超时时重设基准，不连续补发。RPC 和仿真执行时间会降低实际频率，不能保证严格 10 Hz；两次循环间隔超过 2 秒会停止任务。

`hoverAsync()` 没有 `timeout_sec` 参数，清理阶段单独创建 `timeout_value=3` 的 RPC 客户端，避免等待默认的一小时。该值是 msgpack-rpc 超时设置，受事件循环和重试影响，不是精确的墙钟三秒保证。起飞/爬升使用 40 秒 RPC 超时，降落使用 45 秒。失联时仍不能保证悬停命令送达，错误会打印出来。

### 轨迹记录与复测

默认 `--fly` 在当前目录创建 `winter_circle_trajectory.csv`，已有同名文件会拒绝覆盖。每次实验指定新文件名：

```bat
python src/air/winter_circle/winter_circle.py --fly --radius 2 --height 3 --speed 0.5 --laps 1 --log winter_run_01.csv
```

CSV 包含单调时钟经过时间、AirSim 状态时间戳、NED 位置、圆心、目标高度、半径误差、高度误差、累计极角及拟发送速度。正半径误差表示位于圆外；高度误差为 `z-target_z`，正值表示低于目标。行数据在发指令前落盘，表示观测和请求，不证明执行成功；异常时保留已写记录。预览模式不创建 CSV。

离线回归命令（不需要 AirSim）：

```bat
python -m unittest discover -s src/air/winter_circle/tests -v
```

需要在 winter 中另行复测：失联提示、起飞、整圈飞行、终止悬停、可选降落，并提交 CSV 与录屏。当前没有新版在线实测结果，不报告实测轨迹精度，也不将离线检查视为飞行验收。

上一轮修改在 Python 3.12 下运行上述 11 项离线回归，全部通过；包括 RPC 返回 False/异常、周期等待、起飞失败后的悬停尝试、连接失败提示和 CSV 防覆盖。尚未验证全新依赖安装或新版在线飞行。

## 圈数输出和采样周期补充

新版输出包含实际累计圈数与请求圈数。`--laps 0.5` 完成后输出 `Partial arc complete: 0.50/0.50 laps`（实际值可能略有超调），不再使用半圈也叫“一整圈完成”的描述。请求至少一圈时保留 `Circle complete` 前缀，并同样显示实际/请求圈数。

CSV 新增 `loop_interval_s`，记录相邻控制循环开始时间的单调时钟差值；首行是进入第一轮循环距循环计时起点的间隔。它包含上一轮 RPC、处理和等待时间，用于检查实际控制频率，不代表 AirSim 状态的固定采样周期。旧版 CSV 没有该列，可用相邻 `elapsed_s` 相减作同类检查。

CSV 在起飞前创建：起飞失败时可能只留下表头；运行中止时保留已写入的行。这是诊断记录，不代表任务完成。

新版仍需在真实 winter 场景复测。复测时用新日志名（例如 `--log winter_run_02.csv`），随录屏提交 CSV 的数据行数、半径误差最小/最大/平均值、高度误差最大绝对值和实际循环间隔。模拟回放生成的日志不能作为真实飞行证据；此前录屏和终端截图属于旧版本记录。

本轮离线回归共 12 项，全部通过；新增用模拟 RPC 执行实际主程序，覆盖半圈/整圈输出、CSV 周期间隔和停止发送条件。未在 winter 中在线复飞。
