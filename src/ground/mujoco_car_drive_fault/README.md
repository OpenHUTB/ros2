# mujoco_car_drive_fault

MuJoCo 地面两轮差速小车：**9 维位姿/轮速/指令特征 → 纯 numpy MLP → 未来 1.0 s 跑偏概率 → 差速容错补偿**。

车沿直线走，左侧驱动轮的轮毂轴承开始拖滞：轮子转得比指令慢一点，另一侧正常。轮速传感器上几乎看不出来
（速度伺服会把轮速拉回指令值），车却慢慢往一侧偏，几秒后跑出车道。这个模块用位姿时序提前判断"要偏了"，
再靠左右轮差速把车拉回来。三档拖滞阻尼把"轻微拖滞"到"严重拖滞"变成可以对比的实验。

## 与仓库已有模块的区别

| 对比项 | `mujoco_diff_drive_slip_compensation` | `mujoco_car_rollover_guard` | 本模块 |
| --- | --- | --- | --- |
| 故障类型 | 附着不足导致打滑 | 载荷偏高导致侧翻 | **单侧驱动轮拖滞** |
| 可观测信号 | 轮速与车速差明显 | 横滚角持续增大 | **轮速差几乎为 0，只能从偏航率/横向偏移看** |
| 输出 | 补偿后的轮速 | 限速后的油门 | **左右轮差速修正量** |
| 失效后果 | 走不快 | 翻车 | **整车跑出车道** |

## 目录结构

```
src/ground/mujoco_car_drive_fault/
├── main.py                     统一入口（--collect/--train/--eval/--demo/--test/--launch）
├── main.sh / main.bat          脚本入口（无参数时默认 --eval）
├── mujoco_car_drive_fault/
│   ├── config.py               路径解析与全部可调参数（三档阻尼、门控阈值、补偿增益）
│   ├── kinematics.py           纯函数：差速解算为左右轮角速度
│   ├── car_env.py              场景加载、故障注入、步进、位姿/轮速读取
│   ├── features.py             9 维特征构造与定尺度归一化
│   ├── model.py                纯 numpy MLP（加权 BCE + Adam）与 AUC
│   ├── guard.py                风险门控（pass/assist/slow）、差速补偿与横向偏差门限基线
│   ├── scenario.py             工况执行器、油门保持与"跑偏事件"标签
│   ├── sim_bridge.py           ROS 2 仿真桥（发布 /imu、/odom、/chassis_state，订阅 /cmd_vel）
│   └── guard_node.py           ROS 2 失效风险节点（/drive_fault_risk、/guard_state → /cmd_vel）
├── models/                     三个拖滞档位场景（light 0.08 / mid 0.10 / severe 0.12）
├── config/params.yaml          ROS 2 参数
├── launch/main.launch.py       ROS 2 双节点闭环启动文件
├── test/                       49 项单元测试
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
ln -s <本仓库路径>/src/ground/mujoco_car_drive_fault .
cd ~/ros2_ws && colcon build --packages-select mujoco_car_drive_fault --symlink-install
source install/setup.bash
```

## 运行

```bash
python3 main.py --collect      # 采集 12 个工况的拖滞跑偏数据
python3 main.py --train        # 训练跑偏风险网络
python3 main.py --eval         # 三组策略 × 6 工况对比评测（默认命令）
python3 main.py --demo         # 录制演示 gif
python3 main.py --test         # 49 项单测 + 三档冒烟
python3 main.py --test --logic-only   # 未装 MuJoCo 也能跑的纯逻辑单测
bash main.sh --eval            # 等价入口
```

ROS 2 闭环：

```bash
ros2 launch mujoco_car_drive_fault main.launch.py                                  # 默认中档拖滞 0.10 / 0.9 m/s
ros2 launch mujoco_car_drive_fault main.launch.py scene:=car_scene_fault_severe.xml fault_side:=right
```

配置在 `config/params.yaml`，改目标车速、故障侧、拖滞阻尼、运行时长不需要重编译。门控节点由 `/imu` 帧驱动，
一帧传感器算一次修正；`/cmd_vel` 由桥接节点在下一个仿真步执行。

## 实测结果

离线评测（单进程逐帧推进，`main.py --eval`，车速 0.9 m/s，训练时没出现过这个车速）：

| 指标 | 数值 |
| --- | --- |
| 训练集 / 验证集样本 | 2709 / 903（按工况切分 9 / 3） |
| 正样本比例 | 1160 / 3612（32.12%） |
| 模型规模 | 9-16-1，177 个参数，220 轮 |
| 训练集 / 验证集 AUC | 0.9997 / 0.992 |
| 验证集准确率 | 0.966 |
| 未参与训练工况上的风险 AUC | 0.998（阈值 0.60 下准确率 0.962） |

| 策略 | 跑偏率 | 平均最大横向偏差 | 平均补偿帧数 | 首次介入 | 轮速差均值 |
| --- | --- | --- | --- | --- | --- |
| none（不干预） | 100%（6/6） | 1.12 m | 0 | — | 0.08 rad/s |
| rule（横向偏差门限） | 0%（0/6） | 0.19 m | 200 | 1.34 s | 0.21 rad/s |
| nn（1.0 s 前瞻） | 0%（0/6） | 0.40 m | 135 | 2.44 s | 0.35 rad/s |

三组策略里两组都能把车拉回车道，差别在偏差幅值和介入时机：rule 靠"位姿已经越限"持续纠正，介入早但整个
过程都带着偏差；nn 只在风险窗口内动手，介入晚，稳定后偏差也不如 rule 小。这一项如实记录，不修饰成
"神经网络全面更优"。

ROS 2 闭环（Ubuntu 22.04 + Humble，实测）：

| 场景 | 故障侧 / 车速 | 结果 |
| --- | --- | --- |
| mid 0.10（默认） | left / 0.9 m/s | 跑满 6.02 s 未跑偏，最大偏差 0.31 m，末端 +0.11 m |
| severe 0.12 | right / 0.9 m/s | 跑满 6.02 s 未跑偏，最大偏差 0.40 m，末端 +0.23 m |

两条路径不是同一条执行方式：闭环里指令经话题传递、在下一仿真步生效，和离线逐帧推进有时序差，
数值不要求一致。

详细数据、公式与源码解析见教程 `docs/ground/mujoco_car_drive_fault.md`。

## 已知限制

- 拖滞用单轮 hinge 关节的 `damping` 表征，是"轴承/制动拖滞"的简化模型，没有建摩擦、温升与磨损；
- 三档阻尼是离散标定，阻尼再大（≥0.20）会进入"原地打转"区，超出本模块的可补偿范围；
- 只做横向容错，不做故障轮隔离或降速停车；补偿权限有硬上限（差速角速度 ±0.45 × 增益）；
- 平地直线单场景，没有坡道、侧风与弯道；车道边界固定 `y=±1.4 m`；
- 风险标签来自"越界事件窗口"，越界后长时间的回程段不在训练分布内。
