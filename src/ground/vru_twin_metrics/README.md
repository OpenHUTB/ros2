# vru_twin_metrics

行人、儿童和骑行者轨迹的数字孪生评价模块。输入 ground truth 与 twin 输出的
CSV，输出诊断 MOTA、ADE/FDE、FN/FP、ID switch 和每帧数量误差。

## 环境

- Python 3.8+
- 可选：ROS 2 Humble、`rclpy`、`std_msgs`

指标计算不依赖 ROS、CARLA 或 MySQL。

## CSV 格式

```csv
track_id,class,timestamp,x,y,z,yaw,speed,confidence
p1,pedestrian,0.00,0.0,0.0,0.0,0.0,1.2,0.98
c1,child,0.00,3.0,5.0,0.0,90.0,1.0,0.95
b1,cyclist,0.00,8.0,2.0,0.0,180.0,4.0,0.92
```

`track_id,class,timestamp,x,y` 必填；`class` 为 `pedestrian`、`child` 或
`cyclist`。

## 直接运行

```bash
cd src/ground/vru_twin_metrics
python main.py
```

默认使用 `data/vru_gt.csv` 和 `data/vru_pred.csv`，输出 `vru_metrics.json`。

指定文件：

```bash
python main.py --gt gt.csv --pred twin.csv --max-distance 1.5
```

## ROS 2

```bash
ros2 launch vru_twin_metrics main.launch.py \
  ground_truth_csv:=/absolute/vru_gt.csv \
  prediction_csv:=/absolute/vru_pred.csv
ros2 topic echo /vru_twin_metrics/metrics
```

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

## 限制

匹配采用逐帧贪心最近邻；MOTA 是诊断性计数指标，不是完整 CLEAR-MOT。该模块
只提供评价层，不包含 MySQL、地图或 CARLA 场景依赖。
