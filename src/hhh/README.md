# Drone Analyzer ROS 2 Module

本模块用于无人机飞行日志解析与状态识别。

## 1. 目录结构
确保本功能包放置在仓库的 `src/` 目录下，结构如下：
```text
src/drone_analyzer/
├── launch/
│   └── analyzer.launch.py     # 启动文件
├── resource/
│   └── drone_analyzer         # 资源标记文件
├── drone_analyzer/
│   ├── __init__.py
│   └── analyzer_node.py       # 核心业务逻辑代码
├── package.xml                # 包依赖声明
├── setup.cfg                  # 脚本安装配置
└── setup.py                   # Python 编译配置
```
## 2. 编译与运行步骤
```text
```
### 2.1 编译节点
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_analyzer
```
### 2.2 运行节点
```bash
source install/setup.bash
ros2 launch drone_analyzer analyzer.launch.py
```
## 3. 验证节点运行状态
```bash
source ~/ros2_ws/install/setup.bash
ros2 node list
```
