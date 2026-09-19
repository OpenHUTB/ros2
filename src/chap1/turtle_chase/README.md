# 双海龟追逐实验(turtle_chase)

## 1. 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 18.04(实测)/ Ubuntu 20.04 均可 |
| ROS | Melodic(实测)/ Noetic 均可,脚本同时兼容 Python2/3 |
| 依赖功能包 | turtlesim、rospy、geometry_msgs、std_srvs(ROS 自带) |
| 硬件 | 普通PC虚拟机(VMware,2核4G)即可运行 |

## 2. 实验内容

逃亡者 turtle1 以纯跟踪法沿绿色圆形轨迹匀速逃跑;程序通过 `/spawn` 服务
动态生成追逐者 turtle2,以"转向P控制 + 速度P控制"双闭环追赶;相对距离
小于 0.6 m 判定抓住,终端打印抓捕用时与抓捕点,追逐者换洋红色画笔进入
跟随模式。完整实验文档见仓库 `docs/chapter/chap1/turtle_chase.md`。

## 3. 运行步骤

```bash
# 1. 将 src 下的功能包拷入 catkin 工作空间并编译
cp -r src/chap1/turtle_chase ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make
source devel/setup.bash
chmod +x ~/catkin_ws/src/turtle_chase/main.py ~/catkin_ws/src/turtle_chase/scripts/*.py

# 2. 一键运行(自动检测并启动 roscore、turtlesim、两个节点)
rosrun turtle_chase main.py

# 3. 退出: 在 main.py 所在终端按 Ctrl+C, 自动清理全部进程
# 4. 重复实验前重置: rosservice call /reset
```

也可以分 4 个终端手动运行:roscore → `rosrun turtlesim turtlesim_node`
→ `rosrun turtle_chase runner.py` → `rosrun turtle_chase chaser.py`
(新终端需先 `source ~/catkin_ws/devel/setup.bash`)。

## 4. 可调参数(不修改代码即可做对比实验)

```bash
rosrun turtle_chase runner.py _speed:=1.0 _radius:=3.0   # 逃亡者变慢、圆变大
rosrun turtle_chase chaser.py _max_speed:=1.3            # 追逐者变慢, 抓捕用时变长
rosrun turtle_chase chaser.py _catch_dist:=0.8           # 放宽抓捕判定距离
```

## 5. 验证命令

```bash
rosnode list                  # /turtle_runner /turtle_chaser
rostopic list                 # 4个话题: 两个 cmd_vel + 两个 pose
rosservice list | grep turtle # /spawn /clear /turtleN/set_pen
rqt_graph                     # 节点-话题计算图
```

## 6. 文件说明

```
src/chap1/turtle_chase/
├── main.py            # 一键启动入口: 自动起 roscore/仿真器/两个节点
├── package.xml        # 功能包清单与依赖
├── CMakeLists.txt     # catkin 编译配置
└── scripts/
    ├── runner.py      # 逃亡者节点(纯跟踪画圆逃跑, 绿色)
    └── chaser.py      # 追逐者节点(spawn生成+双P控制追赶, 红色→洋红跟随)
```
