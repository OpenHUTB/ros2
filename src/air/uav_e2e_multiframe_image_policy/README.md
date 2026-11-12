# uav_e2e_multiframe_image_policy

多帧图像端到端策略仿真：连续三帧下视灰度图堆叠输入，固定卷积提取中心可见度、横向质心偏差与梯度线索，两层 numpy MLP 直接输出前飞速度与偏航指令，不经过显式感知—规划—控制分层。

## 概览

| 项目 | 内容 |
| --- | --- |
| 话题类别 | 任务 5：端到端模型（图像 → 控制指令，多帧时序堆叠） |
| 平台 | Ubuntu 20.04 ROS Noetic / Ubuntu 22.04 ROS 2 Humble；离线仿真可在 Windows Python 3.13 运行 |
| 语言 | Python 3 |
| 依赖 | rclpy、nav_msgs、std_msgs（ROS 节点）；离线仅依赖 numpy、pygame |
| 运行方式 | `ros2 launch uav_e2e_multiframe_image_policy uav_e2e_multiframe_image_policy.launch.py`；离线 `python main.py --shots` |
| 仿真器 | 内置 numpy 平面运动模型 + pygame 离屏/窗口渲染 |

## 原理

- 机载下视相机以机体为中心渲染 16×12 局部俯视灰度图，帧缓存保留最近 3 帧，不足时重复首帧填充，形成时序堆叠输入。
- 固定 3×3 卷积核提取中心线中心可见度、左右边缘与横向梯度，全局平均后得到 4 维图像线索；两层 MLP 直接回归前飞速度与偏航指令。
- 控制器按指令积分机体位姿，闭环沿正弦参考中心线完成全程跟随；专家参考控制器仅用于校验反馈方向与权重参照。

## 快速运行

### ROS 2 Humble

```bash
colcon build --packages-select uav_e2e_multiframe_image_policy --symlink-install
source install/setup.bash
ros2 launch uav_e2e_multiframe_image_policy uav_e2e_multiframe_image_policy.launch.py
```

### 离线仿真（无需 ROS）

```bash
python main.py --shots    # 离屏运行并保存全过程截图到 docs/img/
python main.py --demo     # 打开观察窗口，查看端到端跟随过程
```

## 测试

```bash
python -m pytest test/ -q
```

## 文件结构

```
uav_e2e_multiframe_image_policy/
├── launch/uav_e2e_multiframe_image_policy.launch.py
├── uav_e2e_multiframe_image_policy/
│   ├── scene.py       # 正弦参考中心线与位姿偏差计算
│   ├── camera.py      # 下视灰度图渲染与三帧历史缓存
│   ├── expert.py      # 专家参考控制器（校验与权重参照）
│   ├── policy.py      # 固定卷积线索提取 + 两层 MLP 端到端策略
│   ├── controller.py   # 指令积分的平面运动控制器
│   ├── runner.py      # 图像-策略-控制闭环与指标统计
│   └── e2e_node.py    # ROS 2 封装节点
├── test/              # 场景、相机、专家、策略、控制器、闭环单元测试
├── main.py            # 离线一键实验入口
├── package.xml
└── setup.py
```

## 许可

MIT
