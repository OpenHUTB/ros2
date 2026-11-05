# ROS Noetic 语音控制小乌龟

教程：docs/voice/pocketsphinx_noetic.md。

仅复制 pocketsphinx 和 robot_voice 到独立 catkin 工作空间 src，容器目录的 CATKIN_IGNORE 和 COLCON_IGNORE 不随包复制。两个源码包均包含 CMakeLists.txt，使用 catkin_install_python 注册程序。

英文控制使用 pocketsphinx/demo/voice_cmd.lm 和 voice_cmd.dic，而不是通用 en-us.lm.bin。前进口令为 forward，不使用 go。robot_voice 以 10Hz 发布指令，默认 1 秒无新命令归零，未知命令立即停止。旧 voice_cmd_vel.py 演示无命令超时，不作为默认控制节点。

原截图只作为旧版识别/订阅记录；本次修订尚待 Ubuntu catkin 构建和麦克风端到端复测。代码与文档由 OpenAI Codex 辅助编写。
