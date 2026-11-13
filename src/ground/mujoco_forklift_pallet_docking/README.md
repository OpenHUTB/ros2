# MuJoCo 叉车相机引导的托盘接近与低速插入

本模块以 MuJoCo 3.12.0 仿真叉车沿直线接近托盘，车载前视相机采集托盘叉孔区域的灰度图，轻量神经网络输出驱动与制动指令，控制叉车低速插入。场景提供多个起始距离和固定随机种子评估。

![叉车前视相机记录](../../../docs/evidence/forklift_pallet_docking/docking.gif)

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- MuJoCo 3.12.0（与随包权重及评估记录一致）
- Python 3、NumPy、Pillow

## 运行

先按 `requirements.txt` 安装依赖，使用 MuJoCo 3.12.0 复现文档中的动力学结果。其他版本的物理与渲染行为不同，不能直接套用本版本的评估数据。

从 ROS 2 工作空间构建并启动：

```bash
colcon build --packages-select mujoco_forklift_pallet_docking
source install/setup.bash
ros2 launch mujoco_forklift_pallet_docking main.launch.py
```

命令行入口为 `ros2 run mujoco_forklift_pallet_docking forklift_pallet_docking --mode evaluate`；保留 `main` 作为兼容入口。

从仓库根目录直接运行：

```bash
python3 src/ground/mujoco_forklift_pallet_docking/main.py --mode evaluate
python3 src/ground/mujoco_forklift_pallet_docking/main.py --mode run --start-x -4 --speed 2
```

启动键盘手动控制（需要交互式终端）：

```bash
python3 src/ground/mujoco_forklift_pallet_docking/main.py --mode manual --start-x -4
```

按 `W` 驱动前进，按 `S` 制动，按空格滑行，按 `Q` 退出。当前仿真模型仅包含纵向驱动轴；终端会实时显示叉车位置和速度。无 TTY 环境可用 `--manual-keys 'wws q'` 执行可重复的按键序列；每个字符对应一个仿真步，空格表示滑行，`q` 退出。

`evaluate` 模式默认载入随 ROS 包安装的网络权重，比较三个起始位置，输出叉齿插入深度、最终速度及是否过度插入，并可用 `--output /tmp/evaluation.json` 保存结果。`train` 模式报告留出集误差；指定 `--model-path` 时同时保存网络权重。

## Ubuntu 容器运行

在本目录构建包含 Ubuntu 22.04、ROS 2 Humble 和 MuJoCo 依赖的镜像：

```bash
docker build -t mujoco-forklift-pallet-docking .
docker run --rm mujoco-forklift-pallet-docking
```

容器启动后由 ROS 2 launch 执行三个起始距离的评估，并在终端输出 JSON 结果。

仓库页面提供[相机实况动图](../../../docs/evidence/forklift_pallet_docking/docking.gif)和[同次录制终态与多起点评估记录](../../../docs/evidence/forklift_pallet_docking/results.json)。从仓库根目录重新录制：

```bash
python3 src/ground/mujoco_forklift_pallet_docking/tools/record_demo.py --output docs/evidence/forklift_pallet_docking/docking.gif --json-output docs/evidence/forklift_pallet_docking/results.json --seed 17 --brake-scale 0.8
```

本次 Windows 录制显式设置 `--brake-scale 0.8`；Ubuntu 容器使用默认 1.0。动图使用灰度相机帧，无需字体；JSON 记录运行命令、MuJoCo 版本、策略来源、种子用途、动图回合终态及多起点评估。每个仿真步为 0.01 秒，动图每帧播放 0.08 秒以便观察。加载已有权重时不会重新训练或计算留出集误差。

## 算法

相机帧转换为 $16\times16$ 灰度像素向量 $\mathbf{x}$。多层感知机预测驱动和制动：

$$
\mathbf h=\tanh(W_1\mathbf{x}+\mathbf b_1),\qquad
\hat{\mathbf a}=\sigma(W_2\mathbf h+\mathbf b_2)
$$

监督目标按叉齿到托盘入口的距离和当前速度生成，制动阈值为 $v^2/8-0.40$，距离不大于该阈值时切换为制动。自动运行通过 `--brake-scale` 显式设置制动力校准系数，Ubuntu 容器默认 1.0。不同起点下的 MuJoCo 相机帧组成训练集，网络通过均方误差优化。

## 文件

- `mujoco_forklift_pallet_docking/model.xml`：叉车车体、叉齿、托盘、相机和驱动器。
- `mujoco_forklift_pallet_docking/policy.py`：图像策略网络和训练标签。
- `mujoco_forklift_pallet_docking/simulation.py`：MuJoCo 渲染、步进与场景评估。
- `main.py`：命令行入口。
- `launch/main.launch.py`：ROS 2 Humble 启动入口。
- `test/`：策略、相机帧与多起点停靠测试。
