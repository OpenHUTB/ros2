# particle_filter_localization

粒子滤波一维蒙特卡洛定位（系统重采样）

## 特点

- **纯 Python 标准库实现**：不依赖 numpy / matplotlib / scipy，沙箱里也能跑。
- 自带单元测试（`tests/`，10 项）。配图直接写到 `docs/img/` 正版位置，不在 `scripts/` 下留运行产物。
- 自带配图生成（手写 zlib PNG 编码 + 5x7 位图字体）。

## 运行

```bash
# 单元测试
python3 tests/test_*.py

# 生成配图 + 打印统计
python3 scripts/main.py --outdir ../../../docs/img/chap6/particle_filter_localization
```

也可以作为 ROS 2 包安装后启动：

```bash
colcon build --packages-select particle_filter_localization
ros2 launch particle_filter_localization particle_filter_localization.launch.py
```

## 文件结构

```
particle_filter_localization/
├── package.xml
├── CMakeLists.txt
├── launch/particle_filter_localization.launch.py
├── requirements.txt
├── scripts/          # 模型、仿真、绘图与入口
└── tests/            # 单元测试
```
