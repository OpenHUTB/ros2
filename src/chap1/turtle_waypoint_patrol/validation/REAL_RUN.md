# 真实运行验证记录（2026-11-04）

## 证据范围

初次运行提供了完整原始压缩日志；新版运行受附件额度限制，仅提供终端摘要和取消测试的完整节点日志。
新版其余六项通过来自该摘要的 7 tests / failures=1；未取得新版完整 CSV 或结果包，不能当作完整归档。
本目录初次 CSV 来自真实 turtlesim 订阅，未生成或插值位姿。

| 版本 | main.py SHA256 | 真实 ROS 结果 |
| --- | --- | --- |
| 初次完整运行 | 094729bff205c2693dfdb2db98ecd32077e1dcc5cbf238f81e5c2babafdab7ea | 7 项中 5 通过，取消后退出及运动中 SIGTERM 退出超时；349.888 秒 |
| 显式释放资源版本（11 月 4 日历史版本） | 5213157363f70fd27e9f9515b4318ac5d6bbcb7b8445fd1f7beaf07cb712f62a | 7 项中 6 通过，取消后退出仍超过 8 秒；58.690 秒 |

8 秒是本项目测试脚本设置的退出门槛，未证实是课程规定。没有改阈值将失败算为通过。

## 11 月 4 日版本唯一失败项

取消后停车断言执行完毕，失败发生在 tearDown 的退出检查。
用户粘贴的节点日志记录：

```text
START pid=3915 sha256=5213157363f70fd27e9f9515b4318ac5d6bbcb7b8445fd1f7beaf07cb712f62a
EXIT cleanup_begin monotonic=4535.767995
EXIT trace_closed monotonic=4535.769765
EXIT handles_released monotonic=4535.875885
EXIT ros_shutdown_begin monotonic=4535.875895
[thread dump: main thread in rospy/rostime.py:277 wallsleep,
 rospy/core.py:629 signal_shutdown; other threads in socket.accept/select]
EXIT ros_shutdown_done monotonic=4559.584686
```

应用清理至句柄释放约 0.108 秒；ROS signal_shutdown 调用约 23.709 秒。
该阶段最终返回，但根本原因尚未确定；不能宣称显式注销已修复退出超时。
现场一次普通 sleep(0.1) 连续 10 次耗时约 0.100～0.107 秒，仅证明当时未复现延迟。

## 初次航点轨迹

`initial_actual_trace.csv` 共 707 行位姿记录，包含 RUNNING、PAUSED、COMPLETED。
末行 completed=3，target_error=0.07852361645933778，低于到达阈值 0.08。
这份轨迹对应初次版本，不能标注为新版轨迹。

## 当时待完成（历史记录）

退出延迟的原因和修复、当前完整结果归档、catkin/roslaunch 构建入口验证、文档网页预览、另一人另一台机器复测。
使用 OpenAI Codex 辅助分析、绘图和文档整理。未伪造 ROS 结果。


## 2026-11-12 审核修订（第一阶段离线检查）

当前代码增加 ROS 全局清理线程的一秒等待上限；超时或异常会返回退出码 2，真实集成仍将其判为失败。
本轮新增 13 项生产控制步骤测试、2 项清理失败测试，连同原 7 项共 22 项离线测试通过。
环境 Python 3.12.14，无 Noetic；Python 3.8 仅做语法兼容检查。
本轮没有新的真实 ROS 通过结论；5/7 和 6/7 均只属于上文记录的历史源码。

## 2026-11-12 后续真实 Noetic 复测

同日随后在官方 Noetic desktop 运行时完成 22/22 Python 3.8 离线测试、三轮各 7/7 真实 turtlesim 测试，以及 catkin 构建安装和安装后的 main.launch 服务/退出验证。
完整证据、运行边界及源码哈希见 [NOETIC_RUN.md](NOETIC_RUN.md)。上面的无 Noetic 和未通过状态仅描述各历史阶段。
