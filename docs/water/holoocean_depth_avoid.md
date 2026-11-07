# 基于深度图的水下前视避障与海底定高巡航

本模块运行于 **HoloOcean2 / ROS 2 Humble** 仿真环境，是水域载具类示例。
它和仓库里已有的 `auv_end2end`（用 RGB 灰度像素判断障碍）不同：
本模块直接消费**深度相机的度量值（米）**，既能做前视避障，又能做离底定高。

## 订阅与发布

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/holoocean/auv0/DepthCamera`（可参数改） | `sensor_msgs/Image` (32FC1, 米) | 前向深度图 |
| 发布 | `/auv/cmd_vel`（可参数改） | `geometry_msgs/Twist` | 6-DOF 速度指令 |

## 状态机

把深度图分成两个 ROI：

- **前向 ROI**（画面中央 30%~70%）：取最近有效深度 `fwd_min`，判断前方障碍；
- **底部 ROI**（画面下方中央）：取中位深度 `bottom_alt`，判断离底高度。

| 状态 | 触发 | 动作 |
|---|---|---|
| GO | `fwd_min > warn_dist` | 按 `cruise_speed` 直行 |
| AVOID | `danger_dist < fwd_min < warn_dist` | 减速，向左右较开阔侧转向 |
| ESCAPE_BACK | `fwd_min < danger_dist` | 向后直退约 1.2s |
| ESCAPE_TURN | 直退结束 | 原地转向约 1.8s（朝开阔侧） |
| AFTER_ESCAPE | 转向结束 | 向前冲约 1.2s 脱离贴墙区 |
| 定高（全程叠加） | `bottom_alt` 偏离 `target_altitude` | 自动上浮/下潜修正 |

脱困链（直退 → 原地转向 → 向前脱离）完成后回到 GO，避免在水下礁石/墙壁前反复顶墙。

## 文件结构

```
src/water/holoocean_depth_avoid/
├── holoocean_depth_avoid/
│   ├── __init__.py
│   └── main.py                       # 节点入口（深度避障 + 定高）
├── launch/
│   └── auv_depth_avoid.launch.py     # launch 启动文件
├── package.xml
├── setup.py                         # ament_python 入口
├── setup.cfg
└── README.md                        # 环境与运行步骤
```

## 运行环境与步骤

Ubuntu 20.04 / 22.04 · ROS 2 Humble · HoloOcean2 · Python 3.10

```bash
# 1) 编译
cd ~/ros2_ws && colcon build --packages-select holoocean_depth_avoid
source install/setup.bash

# 2) 启动 HoloOcean2 带 DepthCamera 的水下世界（见 water 类目环境教程）

# 3) 确认深度话题名，并按需修改 launch 里的 depth_topic
ros2 topic list | grep -i depth

# 4) 启动
ros2 launch holoocean_depth_avoid auv_depth_avoid.launch.py
```

## 运行效果

![holoocean_depth_avoid](../img/water/holoocean_depth_avoid.gif)

## 使用的具体大模型声明

本模块的节点代码、launch、文档由大模型（豆包）辅助生成，作者已逐项核对逻辑与参数，并对提交内容负全部责任。
