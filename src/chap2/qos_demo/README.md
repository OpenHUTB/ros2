# qos_demo

ROS 2 QoS 服务质量策略的行为模型（可靠性 / 历史深度 / 持久性）

## 特点

- **纯 Python 标准库实现**：不依赖 numpy / matplotlib / scipy，沙箱里也能跑。
- 自带单元测试（`tests/`）。
- 自带配图生成（手写 zlib PNG 编码 + 5x7 位图字体）。

## 运行

```bash
# 单元测试
python3 tests/test_*.py

# 生成配图 + 打印统计
python3 scripts/main.py --outdir ../../../docs/img/chap2/qos_demo
```

也可以作为 ROS 2 包安装后启动：

```bash
colcon build --packages-select qos_demo
ros2 launch qos_demo qos_demo.launch.py
```

## 文件结构

```
qos_demo/
├── package.xml
├── CMakeLists.txt
├── launch/qos_demo.launch.py
├── requirements.txt
├── scripts/          # 模型、仿真、绘图与入口
└── tests/            # 单元测试
```
