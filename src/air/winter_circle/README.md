# winter AirSim 圆周控制

教程见仓库 docs/air/winter_circle.md。此脚本直接连接 AirSim RPC，不是 ROS 节点。保留 tutorial_basic/circle_node.cpp 的径向纠偏与切向速度控制思路，独立实现 AirSim 适配。

仅参数预览：python winter_circle.py
实际飞行：python winter_circle.py --fly --radius 2 --height 3 --speed 0.5 --laps 1

需要已有 winter UE4.27 场景和支持 import airsim 的 Python 环境。用户实测终端已输出 Circle complete，录屏和截图见教程；尚无定量轨迹误差数据。

