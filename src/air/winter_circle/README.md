# winter AirSim 圆周控制

教程见仓库 docs/air/winter_circle.md。此脚本直接连接 AirSim RPC，不是 ROS 节点。保留 tutorial_basic/circle_node.cpp 的径向纠偏与切向速度控制思路，独立实现 AirSim 适配。

仅参数预览：python winter_circle.py
实际飞行：python winter_circle.py --fly --radius 2 --height 3 --speed 0.5 --laps 1

需要已有 winter UE4.27 场景和支持 import airsim 的 Python 环境。用户实测终端已输出 Circle complete，录屏和截图见教程；尚无定量轨迹误差数据。


依赖安装、RPC 超时及 CSV 字段详见教程新增的“PR #850 复核后的改进”。
每次实际飞行用 `--log run_01.csv` 指定新文件，默认拒绝覆盖已有日志。
离线检查：`python -m unittest discover -s src/air/winter_circle/tests -v`。
本次改进尚未在 winter 场景重新飞行验证。

## 圈数输出和采样周期补充

新版输出包含实际累计圈数与请求圈数。`--laps 0.5` 完成后输出 `Partial arc complete: 0.50/0.50 laps`（实际值可能略有超调），不再使用半圈也叫“一整圈完成”的描述。请求至少一圈时保留 `Circle complete` 前缀，并同样显示实际/请求圈数。

CSV 新增 `loop_interval_s`，记录相邻控制循环开始时间的单调时钟差值；首行是进入第一轮循环距循环计时起点的间隔。它包含上一轮 RPC、处理和等待时间，用于检查实际控制频率，不代表 AirSim 状态的固定采样周期。旧版 CSV 没有该列，可用相邻 `elapsed_s` 相减作同类检查。

CSV 在起飞前创建：起飞失败时可能只留下表头；运行中止时保留已写入的行。这是诊断记录，不代表任务完成。

新版仍需在真实 winter 场景复测。复测时用新日志名（例如 `--log winter_run_02.csv`），随录屏提交 CSV 的数据行数、半径误差最小/最大/平均值、高度误差最大绝对值和实际循环间隔。模拟回放生成的日志不能作为真实飞行证据；此前录屏和终端截图属于旧版本记录。

本轮离线回归共 12 项，全部通过；新增用模拟 RPC 执行实际主程序，覆盖半圈/整圈输出、CSV 周期间隔和停止发送条件。未在 winter 中在线复飞。
