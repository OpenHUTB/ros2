# line_following_pid

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点，也不发布 ROS 话题。

循线小车的 PID 横向控制 2D 仿真。小车用自行车运动学前进，读取自身相对正弦赛道中心线的横向误差，经带输出限幅的 PID 算出前轮转角，沿赛道行进。

对应文档：docs/ground/line_following_pid.md

## 目录结构

```
line_following_pid/
├── main.py                     # 入口，转发到 line_following_pid.main
├── main.bat                    # Windows 启动脚本
├── main.sh                     # Linux 和 macOS 启动脚本
├── package.xml                 # 包声明，ament_python
├── requirements.txt            # 依赖 pygame 和 pytest
├── setup.py                    # 打包与安装路径
├── setup.cfg
├── README.md
├── resource/line_following_pid  # ament_index 包标记
├── launch/main.launch.py       # ROS 2 launch 占位入口
├── line_following_pid/
│   ├── __init__.py
│   ├── config.py               # 赛道、车辆与 PID 参数
│   ├── pid.py                  # PID 控制器，输出限幅
│   ├── track.py                # 赛道采样、最近点与横向误差
│   ├── car.py                  # 自行车运动学推进
│   └── main.py                 # pygame 渲染与主循环
└── test/test_line_following_logic.py
```

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1
- 安装依赖：

```bash
pip install -r requirements.txt
```

## 运行

```bash
cd src/ground/line_following_pid
python main.py
```

Windows 用 main.bat，Linux 和 macOS 用 main.sh。

## 测试

```bash
cd src/ground/line_following_pid
python -m pytest test/test_line_following_logic.py
```

覆盖：PID 比例项与输出限幅、积分清零、赛道采样、最近点误差为零、跑完整条赛道的横向误差上界。

## 实测结果

本地复跑 529 步跑完整条赛道：起步横向误差 18.12 px，起步时 y 方向平移 25 px；最大绝对横向误差 23.93 px，终点误差 -6.44 px。全程 8 次穿越中心线，后半段误差峰峰值 16.35 px，最后 10 帧仍有小幅漂移。

## 限制

- 纯 2D 几何演示，车辆用简化自行车模型，不接真实定位与控制器话题。
- 赛道固定为正弦曲线，未做鼠标点选交互。
