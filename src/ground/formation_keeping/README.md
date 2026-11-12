# formation_keeping

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点、不发布 ROS 话题；`entry_points` 中的 console_script 仅用于启动 pygame 主循环。

多机器人编队保持 2D 仿真。领航者沿圆周运动，两个跟随者保持在领航者坐标系下的固定偏移，形成三角队形；跟随者用比例控制追期望位置，单帧位移有上限。

对应文档：[`docs/ground/formation_keeping.md`](../../../docs/ground/formation_keeping.md)

## 目录结构

```
formation_keeping/
├── main.py                     # 入口，转发到 formation_keeping.main
├── main.bat / main.sh          # Windows / Linux、macOS 启动脚本
├── package.xml                 # 包声明（ament_python）
├── requirements.txt            # 依赖：pygame、pytest
├── setup.py / setup.cfg        # 打包与安装路径
├── README.md
├── LICENSE
├── resource/formation_keeping  # ament_index 包标记
├── launch/main.launch.py       # ROS 2 launch 占位入口
├── formation_keeping/
│   ├── __init__.py
│   ├── config.py               # 画布、轨迹、队形偏移与控制增益
│   ├── formation.py            # 偏移旋转、期望位置、队形误差、领航者状态
│   ├── team.py                 # 编队状态机与比例控制跟随
│   └── main.py                 # pygame 渲染与主循环
└── tests/test_formation_keeping.py
```

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1
- 安装依赖：

```bash
pip install -r requirements.txt
```

## 运行

```bash
cd src/ground/formation_keeping
python main.py        # Windows 用 main.bat，Linux/macOS 用 main.sh
```

## 测试

```bash
cd src/ground/formation_keeping
python -m pytest tests/test_formation_keeping.py
```

覆盖：偏移旋转、不同朝向下的期望位置、领航者圆周约束、队形误差定义、初始偏离后收敛到阈值以内（含 100 步内误差降至初始一半以下的收敛过程断言），以及收敛后长时间保持队形。

## 误差口径

队形误差为领航者与两个跟随者共 3 个成员各自与期望位置距离的均方根（RMS），分母为成员数 3；领航者自身偏移为 0，其位置偏差也一并计入 RMS。
