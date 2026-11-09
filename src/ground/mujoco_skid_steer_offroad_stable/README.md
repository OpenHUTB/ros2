# mujoco_skid_steer_offroad_stable

MuJoCo 滑移转向越野小车定点巡航仿真（ROS Noetic / catkin 功能包）。
完整教程与实测数据见 <https://openhutb.github.io/ros2/ground/mujoco_skid_steer_offroad/>。

## 依赖

在**本包目录内**执行：

```bash
sudo apt update && sudo apt install -y python3-pip libgl1
python3 -m pip install -r requirements.txt
```

## 构建（在 catkin 工作空间内）

```bash
# 1. 本包目录内：把包拷进工作空间
mkdir -p ~/skid_ws/src
cp -r . ~/skid_ws/src/mujoco_skid_steer_offroad_stable

# 2. 工作空间内：编译
source /opt/ros/noetic/setup.bash
cd ~/skid_ws && catkin_make

# 3. 每个新终端
source ~/skid_ws/devel/setup.bash
rospack find mujoco_skid_steer_offroad_stable
```

## 运行

```bash
# 已 source ~/skid_ws/devel/setup.bash
roslaunch mujoco_skid_steer_offroad_stable main.launch                 # 带 viewer
roslaunch mujoco_skid_steer_offroad_stable main.launch headless:=true  # 无窗口，跑完出图

# 免编译直跑（在本包目录内）
bash run.sh --headless
```

## 参数

| 参数 | 说明 |
| --- | --- |
| `headless:=true` / `--headless` | 不打开 viewer |
| `stabilize:=true` / `--stabilize` | 启用俯仰差速补偿（实验性，默认关闭） |
| `--model PATH` | 指定其他模型文件 |
| `--no-plot` | 不生成曲线图 |
| `--plot-path PATH` | 指定曲线图输出路径，默认包根目录 `traj_plot.png` |

## 文件

```text
CMakeLists.txt   catkin 构建（安装脚本、模型、launch、依赖清单）
package.xml      包元信息（format 2）
launch/          main.launch（roslaunch 入口）
models/          skid_robot_offroad.xml（车体、起伏地面、相机、执行器）
scripts/         main.py（主程序）、stable_controller.py（可选补偿器）
run.sh           一键启动（设置 LIBGL_ALWAYS_SOFTWARE=1）
requirements.txt pip 依赖
```
