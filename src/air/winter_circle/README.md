# winter AirSim 圆周控制

教程见仓库 docs/air/winter_circle.md。此脚本直接连接 AirSim RPC，不是 ROS 节点。保留 tutorial_basic/circle_node.cpp 的径向纠偏与切向速度控制思路，独立实现 AirSim 适配。

仅参数预览：python winter_circle.py
实际飞行：python winter_circle.py --fly --radius 2 --height 3 --speed 0.5 --laps 1

需要已有 winter UE4.27 场景和支持 import airsim 的 Python 环境。用户实测终端已输出 Circle complete，录屏和截图见教程；尚无定量轨迹误差数据。


依赖安装、RPC 超时及 CSV 字段详见教程新增的“PR #850 复核后的改进”。
每次实际飞行用 `--log run_01.csv` 指定新文件，默认拒绝覆盖已有日志。
离线检查：`python -m unittest discover -s src/air/winter_circle/tests -v`。
本次改进尚未在 winter 场景重新飞行验证。
