# Quadruped MuJoCo RL Control (ROS)

## 运行环境
- Ubuntu 20.04 (VMware 虚拟机)
- ROS Noetic
- Python
- MuJoCo 物理引擎
- Stable-Baselines3 (PPO)

## 运行步骤

1. 编译工作空间并刷新环境：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
2. 启动 ROS 节点管理器：
   ```bash
   roscore
   ```

    (保持运行，不要关闭该终端)

3. 另开一个新的终端，启动仿真测试节点：
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun quadruped_mujoco_rl main_test.py
   ```

## 补充：站立与步态控制（不需要 ROS，Ubuntu 22.04 + MuJoCo 3.x 可直接跑）

上面的步骤依赖 ROS Noetic。为了在 ROS 2 Humble 或纯 Python 环境下也能复现，
`scripts/` 下新增了四个脚本（原有 `main_test.py` / `main_train.py` 未改动）：

| 文件 | 作用 |
| --- | --- |
| `locomotion_core.py` | 控制核心库：站立/姿态保持 + 参数化对角步态 + 指标统计 |
| `run_locomotion.py` | 跑单次实验并打印指标（可导出逐帧 CSV） |
| `tune_gait.py` | 步态参数搜索，输出"参数-稳定性"对照表 |
| `locomotion_node.py` | ROS 2 节点：发布 `/joint_states`、订阅 `/cmd_vel` |

```bash
pip install mujoco
cd scripts

python3 run_locomotion.py --mode stand --seconds 3          # 站立基线
python3 run_locomotion.py --mode stand --seconds 3 --disturbance 60   # 抗扰
python3 run_locomotion.py --mode trot --seconds 8           # 对角步态（约 0.79 m/s）
python3 tune_gait.py --trials 150 --seconds 4               # 参数搜索

source /opt/ros/humble/setup.bash                           # ROS 2 入口
python3 locomotion_node.py
```

实测数据、参数扫描结果与未验证项见
[文档第八节](https://openhutb.github.io/ros2/quadruped_mujoco_rl/)。
