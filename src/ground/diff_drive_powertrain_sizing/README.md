# diff_drive_powertrain_sizing

差速底盘驱动系统选型核算。由「上坡 + 匀加速 + 巡航风阻」保守工况算轮边转矩、轮速与电功率，
对比直接驱动与按电机额定转速配减速箱两种方案的电机需求，并给持续/峰值转矩判定。

公式与判定见 [文档页](../../../docs/ground/diff_drive_powertrain_sizing.md)。

## 依赖

运行与测试仅需 Python 3.8+ 标准库（测试文件无 pytest 也能直接跑）。

## 运行

```bash
cd src/ground/diff_drive_powertrain_sizing
python3 main.py --output outputs
python3 main.py --mass 20 --grade 15 --motor-cont-torque 2.0 --output outputs
```

输出 `outputs/report.json`（已进 `.gitignore`）。

## 测试

```bash
python3 test/test_diff_drive_powertrain_sizing.py   # 8 项
python3 -m pytest test -v                            # CI 用
```

## ROS 2 包装

```bash
colcon build --packages-select diff_drive_powertrain_sizing
source install/setup.bash
ros2 launch diff_drive_powertrain_sizing main.launch.py mass:=20 grade:=15
```

`launch/main.launch.py` 用 `ExecuteProcess` 调 CLI，不需要 rclpy；尚未在 Ubuntu + Humble 上实跑，
见 `validation/offline_validation.json`。
