# uav_mujoco_depth_avoidance —— 无人机深度相机神经网络避障

> 课程任务（1）物理仿真 + 键盘运动控制，任务（2）传感器感知 + 给定轨迹的控制。
> MuJoCo 真实刚体四旋翼 + 64×64 机载深度相机 + 纯 numpy CNN（输出左/中/右三扇区通行度）
> + 显式控制律闭环飞行。不用 Gazebo，不用 GPU，不依赖 PyTorch/TensorFlow。

完整原理推导、算法流程、源码解析与多场景性能评价：
[`docs/air/uav_mujoco_depth_avoidance.md`](../../../docs/air/uav_mujoco_depth_avoidance.md)

## 运行环境

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Windows 10/11、Ubuntu 20.04 / 22.04 |
| Python | 3.8+（已在 3.10.6 验证） |
| 依赖 | `numpy`、`mujoco>=3.0`、`matplotlib`、`pillow`；键盘模式另需 `pygame` |
| ROS（可选） | ROS 2 Humble / ROS Noetic，仅 `--ros` 模式需要 |
| 硬件 | 普通 CPU；`--demo` 全流程约 14 s |

## 运行步骤

### 1. 安装依赖

```bash
cd src/air/uav_mujoco_depth_avoidance
pip3 install -r requirements.txt
```

### 2. 训练 + 三场景闭环（默认）

```bash
python3 main.py --demo --output-dir results
# Windows 也可直接运行 main.bat
```

生成 `results/training_curve.png`、`results/prediction.png`、
`results/flight_{slalom,gate,clutter}.gif`（1.7–2.4 MB）与 `results/metrics.json`。

### 3. 键盘遥操作（任务 1）

```bash
python3 main.py --teleop
```

`W/S` 调前向速度，`A/D` 调偏航角速度，`空格` 悬停，`R` 重开，`ESC` 退出。
窗口左侧追尾视角、右侧深度相机、底部实时显示网络预测。

### 4. 单元测试

```bash
python3 main.py --test        # 16 passed，约 1 s
python3 -m pytest -q test
```

### 5. ROS 2 运行

```bash
colcon build --packages-select uav_mujoco_depth_avoidance --symlink-install
source install/setup.bash
ros2 launch uav_mujoco_depth_avoidance main.launch.py
```

话题：发布 `/uav/depth`、`/uav/sector_clearance`、`/uav/odom`；订阅 `/uav/cmd_vel`、`/uav/scene`。

## 目录结构

```
uav_mujoco_depth_avoidance/
├── main.py / main.sh / main.bat      # 课程约定主入口（main.*）
├── launch/main.launch.py             # ROS 2 launch 入口
├── config/params.yaml                # 场景、频率、是否用策略
├── models/uav_depth_scene.xml        # MuJoCo 场景 + 四旋翼 + 深度相机
├── uav_nav/
│   ├── env.py                        # 物理步进、深度渲染、三场景布局
│   ├── data.py                       # 数据集与网络输入
│   ├── net.py                        # im2col 卷积 + 手写反向传播 + Adam
│   ├── train.py                      # 训练、出图、GIF
│   ├── policy.py                     # 控制律与闭环评价
│   ├── teleop.py                     # 键盘遥操作
│   └── node.py                       # ROS 2 节点
├── test/test_uav_nav.py              # 16 项离线测试
└── requirements.txt
```

## 典型输出

```text
[slalom]  成功=True 碰撞=False 步数=221 前进=13.60 m 最小余量=0.074 预测误差=0.043 路径长=14.12 m
[gate]    成功=True 碰撞=False 步数=293 前进=13.61 m 最小余量=0.061 预测误差=0.038 路径长=14.15 m
[clutter] 成功=True 碰撞=False 步数=229 前进=13.61 m 最小余量=0.060 预测误差=0.043 路径长=14.15 m

参数数量            : 19667
验证 MAE(左/中/右)  : 0.033 / 0.047 / 0.052
多场景成功率        : 9/9
```
