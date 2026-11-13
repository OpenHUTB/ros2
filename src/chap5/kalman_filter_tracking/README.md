# kalman_filter_tracking

卡尔曼滤波一维匀速目标跟踪（手写 2x2 矩阵运算）

## 特点

- **纯 Python 标准库实现**：不依赖 numpy / matplotlib / scipy，沙箱里也能跑。
- 自带单元测试（`tests/`）。
- 自带配图生成（手写 zlib PNG 编码 + 5x7 位图字体）。

## 运行

```bash
# 单元测试
python3 tests/test_*.py

# 生成配图 + 打印统计
python3 scripts/main.py --outdir ../../../docs/img/chap5/kalman_filter_tracking
```

也可以作为 ROS 2 包安装后启动：

```bash
colcon build --packages-select kalman_filter_tracking
ros2 launch kalman_filter_tracking kalman_filter_tracking.launch.py
```

## 文件结构

```
kalman_filter_tracking/
├── package.xml
├── CMakeLists.txt
├── launch/kalman_filter_tracking.launch.py
├── requirements.txt
├── scripts/          # 模型、仿真、绘图与入口
└── tests/            # 单元测试
```
