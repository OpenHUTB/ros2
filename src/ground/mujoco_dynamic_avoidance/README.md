# mujoco_dynamic_avoidance

MuJoCo 动态障碍局部避障仿真。

## 环境

- Ubuntu 20.04
- Python 3.8
- MuJoCo 2.3.7（pip install mujoco==2.3.7）

## 安装

    pip install mujoco==2.3.7

## 运行

    cd src/ground/mujoco_dynamic_avoidance
    python3 main.py

## 预期现象

1. 橙色自车从道路左端起点出发，朝右端终点行驶；
2. 路上三个蓝色障碍沿横向往复穿行；
3. 自车检测到近处障碍后绕行避让，奇偶障碍交替从道路两侧绕过，避开后回到通往终点的方向；
4. 最终停在终点附近，全程不撞上障碍。

## 已知限制

- 用人工势场做局部避障，目标吸引加障碍排斥，没有全局路径规划；
- 障碍是预设的正弦往复运动，不是真实传感器感知；
- 车辆用三个 slide/hinge 关节直接控制位置，不是真实阿克曼转向；
- 势场法在特殊队形下可能陷入局部最小点。
