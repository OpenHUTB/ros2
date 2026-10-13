# ROS Noetic 语音控制示例

教程见 `docs/chapter/chap8.md`。

本目录收录用户提供的 pocketsphinx 和 robot_voice 源码，保留原有包元数据。仅将这两个子目录复制到独立 catkin 工作空间的 src 中，不要复制本目录的 CATKIN_IGNORE。不要与第五章同名的 robot_voice 包放在同一工作空间。

英文模型安装 pocketsphinx-en-us；可选中文模型单独存放于 ~/pocketsphinx_models，不在仓库内重复存储。原有演示 launch 文件是上游遗留示例，本章复现采用教程中的 rosrun 命令。

robot_voice 的 package.xml 许可证仍为 TODO，尚未替作者指定许可。
