# rtk_ambiguity_resolution

单频 RTK 的双差整周模糊度固定：浮点解 → LAMBDA 整数去相关 → 整数搜索 → ratio 检验。
模块给出 float/fixed 基线误差、固定成功率与错误固定率随观测窗口长度的变化，全部由确定性仿真驱动。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select rtk_ambiguity_resolution
source install/setup.bash
ros2 launch rtk_ambiguity_resolution main.launch.py windows:=20
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/rtk/observation` | `std_msgs/String` | 输出（仿真） | 双差相位/码观测与几何（JSON） |
| `/rtk/status` | `std_msgs/String` | 输出（解算） | 基线、固定标志、ratio、模糊度 |

## 离线实验

```bash
cd src/ground/rtk_ambiguity_resolution
python3 main.py --out results --figure-dir ../../../docs/img/ground/rtk_ambiguity_resolution
python3 -m unittest discover -s tests -v
```

## 主要结果（20 历元/次固定，20 次）

| 指标 | 数值 |
|---|---|
| 浮点解基线 RMSE | 160.51 mm |
| 固定解基线 RMSE | 0.89 mm |
| 整数固定正确率 | 100 % |
| 错误固定率 | 0 % |
| 去相关后条件数 | 24855 → 7.87 |

详见 `docs/ground/rtk_ambiguity_resolution.md`。
