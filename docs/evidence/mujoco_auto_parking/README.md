# 实测证据

本目录保存自动泊车模块的真实运行证据，便于课程检查和跨机器复现。

## 一键回归

`verification/summary.txt`：

```text
task1 exit=0
sensors exit=0
networks exit=0
trajectory exit=0
task3 exit=0
task4 exit=0
```

各入口完整日志：

| 文件 | 内容 |
| --- | --- |
| `verification/task1.log` | 任务①模型、前进/后退/转向、键位、Watchdog |
| `verification/sensors.log` | 相机/深度/激光雷达/感知网络 |
| `verification/networks.log` | 感知精度、控制网络精度、零权值消融 |
| `verification/trajectory.log` | S 曲线闭环跟踪 |
| `verification/task3.log` | 建图、Neural A*、双目标导航闭环 |
| `verification/task4.log` | 图像端到端闭环与零权值消融 |

## 训练日志

| 文件 | 内容 |
| --- | --- |
| `perception_training.csv` | 车位感知网络逐 epoch 训练/验证损失 |
| `trajectory_training.csv` | 轨迹控制网络逐 epoch 训练/验证损失 |
| `planner_training.csv` | Neural A* 引导场逐 epoch 损失 |
| `e2e_training.csv` | 端到端 CNN 逐 epoch 损失 |

## Word 导出

`mujoco_auto_parking_report.docx`：由 `tools/export_word.py` 将模块首页和四篇文档合并导出，`python-docx` 重新打开校验为 264 段、21 张表。LaTeX 公式以文本保留。

## 演示 GIF

见 `docs/img/ground/mujoco_auto_parking/`：

| 文件 | 内容 |
| --- | --- |
| `task1_teleop.gif` | 任务①前进/转向/停止 |
| `task2_s_curve.gif` | 任务②神经网络 S 曲线跟踪 |
| `task3_navigation.gif` | 任务③建图 + Neural A* 导航 |
| `task4_end_to_end.gif` | 任务④图像端到端控制 |

task1 由 ScreenToGif 录制，task2-4 由 `tools/make_gifs.py` 从真实 MuJoCo 渲染帧生成；四个 GIF 为 3.51/1.74/2.22/1.75 MB，均 <10 MB。
