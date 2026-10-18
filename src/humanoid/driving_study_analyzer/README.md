# driving_study_analyzer

DReyeVR 驾驶研究 CSV 的离线分析模块。它读取
`<session>.csv` 和 `<session>_gaze.csv`，输出驾驶时长、有效采样率、速度/控制
活跃度、注视有效率、扫视事件和信号灯注意力等指标。

## 环境

- Python 3.8+
- 可选：ROS 2 Humble、`rclpy`、`std_msgs`

核心分析不需要 ROS 或 CARLA。

## 直接运行

```bash
cd src/humanoid/driving_study_analyzer
python main.py
```

默认读取 `data/sample_session.csv` 和 `data/sample_session_gaze.csv`，生成
`driving_study_report.json`。

指定文件：

```bash
python main.py --driving session.csv --gaze session_gaze.csv --out report.json
```

## ROS 2

```bash
ros2 launch driving_study_analyzer main.launch.py \
  driving_csv:=/absolute/session.csv \
  gaze_csv:=/absolute/session_gaze.csv \
  output_json:=/absolute/report.json
ros2 topic echo /driving_study_analyzer/report
```

报告使用 `transient_local` QoS，节点启动后发布的最后一份报告会保留，因此
`ros2 topic echo` 晚于节点启动也可以收到。

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

## 限制

当前没有把分析结果回放到肌肉骨骼模型；该模块先提供稳定的数据契约和指标。
CSV 中没有前车距离，因此不计算 TTC/PET，也不把“红灯停车候选”解释为因果标签。
