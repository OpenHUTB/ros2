# 第三方来源

`models/community_quadrotor.xml` 原样取自 OpenHUTB/ros2 社区 PR #178（作者 eeexty），上游 `package.xml` 声明 MIT。精确仓库、提交、路径及 SHA256 见 `models/source.json`。该 PR 在本项目开发时尚未合并，不能表述为已成为上游正式功能；本项目带有原样模型，因此运行不依赖该 PR 是否合并。模型文件原有注释完整保留。

本项目运行时新增四个球形起落架接触点和地面，并调整质量、物理步长、接触软度；未复制上游键盘控制器、数据、实验图或指标。

MuJoCo 由 Google DeepMind 开发，采用 Apache-2.0 许可，通过 pip 安装，不在本项目中重新分发其二进制。PyTorch、NumPy、Matplotlib、ROS2 分别遵循其自身许可。
