# diff_drive_wall_following — 差速小车沿墙行驶

订阅 `/scan`（`sensor_msgs/LaserScan`），取正前、正右、右前、右后、正左五路距离，
用「右侧距离误差 + 车身与墙夹角误差」计算速度指令并发布到 `/cmd_vel`
（`geometry_msgs/Twist`）。正前方太近时进入转角状态，原地左转，转过墙角后继续跟随。
配套教程见 [docs/ground/diff_drive_wall_following.md](../../docs/ground/diff_drive_wall_following.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（ROS 节点） |
| ROS | ROS Noetic（Python 3） |
| 依赖 | rospy、geometry_msgs、sensor_msgs |
| 机器人 | 带 2D 激光雷达的差速底盘（如 TurtleBot3 burger） |
| 离线验证 | Windows / Linux，Python 3 + numpy + matplotlib（不需要 ROS） |

## 运行步骤

### 方式一：ROS + Gazebo（以 TurtleBot3 为例）

```bash
# 1. 拷贝到工作空间并编译
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/ground/diff_drive_wall_following ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/diff_drive_wall_following/main.py
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 2. 启动带激光雷达的仿真世界
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 3. 确认激光话题有数据
rostopic hz /scan

# 4. 启动沿墙节点
roslaunch diff_drive_wall_following main.launch
```

把小车放在离墙约 0.5 m、车头与墙平行的位置再启动，跟随效果最好。

### 方式二：离线仿真（不需要 ROS，用于快速验证控制逻辑）

```bash
cd <本仓库>
python -m pip install numpy matplotlib
python src/ground/diff_drive_wall_following/offline_sim.py
```

脚本在 6 m × 5 m 的矩形房间里模拟 1° 分辨率激光和差速小车运动学，
让小车沿右侧墙逆时针绕行一圈，输出三张图和一组指标。

## 运行测试

```bash
cd <本仓库>
python3 src/ground/diff_drive_wall_following/test/test_diff_drive_wall_following_local.py
```

测试不需要 ROS / Gazebo，覆盖沿墙控制、墙角状态切换、速度限幅和工程文件；
GitHub Actions 会在 `src/ground/diff_drive_wall_following/**` 变更时自动运行该测试并构建文档。

## 参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~target_distance` | 0.5 | 期望的右侧墙距离（m） |
| `~k_distance` | 1.6 | 距离误差到角速度的比例系数 |
| `~k_align` | 1.2 | 车身与墙夹角误差到角速度的比例系数 |
| `~max_angular` | 0.9 | 最大角速度（rad/s） |
| `~base_speed` | 0.35 | 沿墙时的基础线速度（m/s） |
| `~turn_speed` | 0.8 | 墙角原地转向角速度（rad/s） |
| `~front_block_distance` | 0.75 | 前方小于该距离判定为墙角（m） |
| `~front_clear_distance` | 1.1 | 前方大于该距离判定转角完成（m） |
| `~scan_timeout` | 0.5 | 激光超时（s），超时后停车 |

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` 线速度、`angular.z` 角速度 |

## 预期效果

小车保持右侧墙约 0.5 m 的间距前进，遇到墙角时原地左转，转过 90° 后继续跟随下一面墙，
最终绕房间一圈回到起点；离线仿真中平均跟墙误差约 3 cm（详细数据见文档）。
