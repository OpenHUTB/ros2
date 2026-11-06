# Underwater Sonar Mapping

本模块用确定性合成数据演示水下三维地形、障碍物和五束声呐扇形探测的可视化。
数据由参数化海底曲面生成，不连接真实声呐设备。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- NumPy
- Matplotlib

## 安装与启动

```bash
cd ~/ros2_ws/src
cp -r /path/to/ros2/src/underwater_sonar_mapping .
cd ..
colcon build --packages-select underwater_sonar_mapping
source install/setup.bash
ros2 launch underwater_sonar_mapping main.launch.py
```

无图形界面时可以直接生成 PNG：

```bash
python3 -m underwater_sonar_mapping.scene \
  --output docs/img/underwater_sonar_3d_mapping_run.png
```

也可以从 ROS 参数指定保存位置：

```bash
ros2 launch underwater_sonar_mapping main.launch.py \
  save_path:=$HOME/underwater_sonar.png
```

## 测试

```bash
python3 -m pytest tests -q
```

测试会验证场景尺寸、曲面范围和障碍物数量，并在无显示环境下渲染 PNG。