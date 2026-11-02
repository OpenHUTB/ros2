# mujoco_car_rollover_guard

MuJoCo 地面两轮差速小车：**9 维 IMU 与指令特征 → 纯 numpy MLP → 未来 0.5 s 翻车概率 → 主动限速**。

车往哪走由上游决定，这个模块只回答一个问题：按当前姿态和这条速度指令继续走，半秒内会不会翻。
风险高就压低油门，很高就直接制动。三档顶部载荷高度把"重心高一点就更容易翻"变成可以对比的实验。

## 与仓库已有模块的区别

| 对比项 | `mujoco_car_avoidance`（规则基线） | 本模块 |
| --- | --- | --- |
| 关注对象 | 前方障碍物距离 | **车身横向稳定性（侧倾/翻车）** |
| 决策方式 | 距离分级状态机 | **神经网络预测 0.5 s 内翻车概率** |
| 输出 | 转向 + 油门 | **风险概率 + 限速后的油门** |
| 是否可独立挂在别的控制后面 | 否（自带决策） | **是（只改速度，不改转向）** |
| 运行前提 | mujoco + numpy | mujoco + numpy |

底盘物理直接沿用规则基线已经调通的方案（速度执行器、静定三点支撑、2 ms 步长 implicitfast 积分），
只把轮距收窄到 0.18 m、后支撑轮放下到 -0.018 m，并在车顶加一块可调高度的 1.8 kg 载荷。

## 目录结构

```
src/ground/mujoco_car_rollover_guard/
├── main.py                     统一入口（--collect/--train/--eval/--demo/--test/--launch）
├── main.sh / main.bat          脚本入口（无参数时默认 --eval）
├── mujoco_car_rollover_guard/
│   ├── config.py               路径解析与全部可调参数
│   ├── kinematics.py           纯函数：差速解算为左右轮角速度
│   ├── car_env.py              场景加载、步进、IMU 读取、倾角与翻车判定
│   ├── features.py             9 维特征构造与定尺度归一化
│   ├── model.py                纯 numpy MLP（加权 BCE + Adam）与 AUC
│   ├── guard.py                风险门控（pass/slow/brake）与横滚角门限基线
│   ├── scenario.py             工况执行器与"未来是否翻车"标签
│   ├── sim_bridge.py           ROS 2 仿真桥（发布 /imu、/odom、/chassis_state，订阅 /cmd_vel）
│   └── guard_node.py           ROS 2 风险节点（/rollover_risk、/guard_state → /cmd_vel）
├── models/                     三个载荷高度场景（low 0.16 / mid 0.26 / high 0.36 m）
├── config/params.yaml          ROS 2 参数
├── launch/main.launch.py       ROS 2 双节点闭环启动文件
├── test/                       38 项单元测试
├── tools/check_scene.py        场景结构自检
├── requirements.txt            Python 运行依赖
└── results/                    数据集、权重、指标 json、曲线（随包安装权重；演示 gif 由 --demo 生成，不入库）
```

## 运行环境

| 组件 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（虚拟机，实测通过）；Windows 10/11 本地 Python 环境可跑仿真与出图 |
| Python | 3.10（虚拟机 3.10.12，本地 3.10.11） |
| MuJoCo | 3.16.0（本地实测；只用稳定接口，3.10 及以上应可用） |
| numpy | 1.26.x |
| 可选依赖 | `matplotlib` 画评测曲线、`imageio` + `pillow` 合成演示 gif；缺失时自动跳过绘图，不算失败 |
| ROS | ROS 2 Humble（仅 `--launch` 与 ROS 节点需要） |
| 显卡 | 不需要；仿真与出图默认离屏无窗口 |

## 安装

```bash
# 系统依赖
sudo apt update && sudo apt install -y python3-pip libgl1

# Python 依赖
python3 -m pip install -r requirements.txt

# ROS 2 功能包（只跑 main.py 时不需要）
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s <本仓库路径>/src/ground/mujoco_car_rollover_guard .
cd ~/ros2_ws && colcon build --packages-select mujoco_car_rollover_guard --symlink-install
source install/setup.bash
```

## 运行

```bash
python3 main.py --collect      # 采集 27 个工况的侧翻数据
python3 main.py --train        # 训练风险网络
python3 main.py --eval         # 三组策略 × 9 工况对比评测（默认命令）
python3 main.py --demo         # 录制演示 gif
python3 main.py --test         # 38 项单测 + 三场景冒烟
python3 main.py --test --logic-only   # 未装 MuJoCo 也能跑的纯逻辑单测
bash main.sh --eval            # 等价入口
```

ROS 2 闭环：

```bash
ros2 launch mujoco_car_rollover_guard main.launch.py                    # 默认中载荷 0.26 m / 1.4 m/s
ros2 launch mujoco_car_rollover_guard main.launch.py scene:=car_scene_payload_high.xml v_target:=1.1
```

配置在 `config/params.yaml`，改目标车速、转向斜率、运行时长不需要重编译。门控节点由 `/imu` 帧驱动，
一帧传感器算一次指令；`/cmd_vel` 由桥接节点在下一个仿真步执行。

## 实测结果

离线评测（单进程逐帧推进，`main.py --eval`）：

| 指标 | 数值 |
| --- | --- |
| 训练集 / 验证集样本 | 6831 / 2325（按工况切分 20 / 7） |
| 验证集 AUC | 0.979 |
| 未参与训练工况上的风险 AUC | 0.994 |
| 翻车率（none / rule / nn） | 44% / 44% / 11% |
| 平均最大横滚（none / rule / nn） | 29.8° / 29.7° / 16.7° |

ROS 2 闭环（Ubuntu 22.04 + Humble，实测）：

| 场景 | 目标车速 | 结果 | 最大倾角 | 门控动作 |
| --- | --- | --- | --- | --- |
| 中载荷 0.26 m（默认） | 1.4 m/s | 连跑 3 次均未翻 | 28.8° | 制动 7 帧、减速 49 帧 |
| 中载荷 0.26 m | 1.1 m/s | 未翻 | 16.8° | 未触发 |
| 低载荷 0.16 m | 1.4 m/s | 未翻 | 13.2° | 触发减速 |
| 高载荷 0.36 m | 1.1 m/s | 5.36 s 侧翻 | 56.6° | 减速、制动后仍翻 |

两条路径不是同一条执行方式：闭环里指令经话题传递、在下一仿真步生效，和离线逐帧推进有时序差，
数值不要求一致。

详细数据、公式与源码解析见教程 `docs/ground/mujoco_car_rollover_guard.md`。

## 已知限制

- 三档载荷是离散几何改动，没有验证载重未知的情形；
- 平地单场景，没有坡道、侧风与低附着路面；
- 翻车判据（总倾角 ≥ 55°）依赖当前底盘参数，改轮距或质心后需要重新标定；
- 门控只限速不修正转向。
