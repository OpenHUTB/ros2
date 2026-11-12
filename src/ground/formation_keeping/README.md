# formation_keeping

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点，也不发布 ROS 话题。entry_points 中的 console_script 仅用于启动 pygame 主循环。

多机器人编队保持 2D 仿真。领航者沿圆周运动，两个跟随者保持在领航者坐标系下的固定偏移，形成三角队形。跟随者用比例控制追期望位置，单帧位移有上限。

对应文档：docs/ground/formation_keeping.md

## 目录结构

```
formation_keeping/
├── main.py                     # 入口，转发到 formation_keeping.main
├── main.bat                    # Windows 启动脚本
├── main.sh                     # Linux 和 macOS 启动脚本
├── package.xml                 # 包声明，ament_python
├── requirements.txt            # 依赖 pygame 和 pytest
├── setup.py                    # 打包与安装路径
├── setup.cfg
├── README.md
├── LICENSE
├── resource/formation_keeping   # ament_index 包标记
├── launch/main.launch.py       # ROS 2 launch 占位入口
├── formation_keeping/
│   ├── __init__.py
│   ├── config.py               # 画布和轨迹，队形偏移与控制增益
│   ├── formation.py            # 偏移旋转，期望位置，队形误差，领航者状态
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
python main.py
```

Windows 用 main.bat，Linux 和 macOS 用 main.sh。

只跑固定帧数并做无头渲染，用于生成文档截图：

```bash
cd src/ground/formation_keeping
python main.py --frames 340
```

指定 --frames 时使用 SDL dummy 驱动，不打开窗口，跑完指定帧数即退出。

## 长时运行

主循环本身不设退出条件，轨迹与误差记录按窗口截断：领航者轨迹与两个跟随者轨迹用 deque 限长 3000 点，队形误差保留最近 3000 条。60 FPS 下窗口约 50 秒，内存占用与每帧绘制开销不随运行时长增长。

## launch 入口

launch/main.launch.py 用 ExecuteProcess 启动 console_script，本包不创建 rclpy 节点，因此不会被注入 Node 的 --ros-args 参数，该 launch 只作为仓库约定的占位入口。

## 构建与依赖声明

package.xml 声明 ament_python 构建类型，并用 exec_depend 声明 python3 与 python3-pygame，走 rosdep 安装时会一并装 pygame。setup.py 补齐 maintainer、maintainer_email、description 与 license 元数据，colcon build 时不再报元数据缺失警告。

## 测试

```bash
cd src/ground/formation_keeping
python -m pytest tests/test_formation_keeping.py
```

覆盖：偏移旋转，不同朝向下的期望位置，领航者圆周约束，队形误差定义，初始偏离后收敛到阈值以内。其中包含 100 步内误差降到初始一半以下的收敛过程断言，并验证收敛后能否长时间保持队形。

## 误差口径

队形误差为领航者与两个跟随者共 3 个成员各自与期望位置距离的均方根，分母为成员数 3。领航者自身偏移为 0，其位置偏差也一并计入 RMS。
