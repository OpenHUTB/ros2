# 提交说明 (SUBMISSION_GUIDE)

本模块 **mujoco_musculoskeletal_knee_tracking**（膝关节肌肉骨骼仿真 + 轨迹跟踪控制）已完成开发与本地验证。以下是把成果提交到 `OpenHUTB/ros2` 仓库（作业）的操作步骤。

## 一、成果清单

```
mujoco_musculoskeletal_knee_tracking/
├── package.xml                     # ROS2 ament_python 包描述
├── setup.py / setup.cfg            # 构建配置
├── resource/                       # ament 资源索引
├── config/params.yaml              # 参数(轨迹/PID)
├── launch/main.launch.py           # launch 入口
├── mujoco_musculoskeletal_knee_tracking/
│   ├── __init__.py
│   ├── main.py                     # 主节点(轨迹跟踪控制)
│   ├── controller.py               # PD→激活 纯逻辑(单元测试)
│   └── model/muscle_knee.xml       # 膝关节肌肉骨骼 MJCF 模型
├── test/test_tracking.py           # 单元测试
├── simulate_tracking.py            # 离线演示: 生成 GIF/评价图/指标
├── validate_model.py               # 模型本地验证
├── README.md                       # 双语说明
└── docs_musculoskeletal_knee_tracking.md   # MkDocs 教程页(待复制)
```

已生成并可提交的运行效果：`tracking.gif`(0.98MB)、`tracking_eval.png`、`tracking_metrics.txt`、`model_render.png`。

## 二、提交到 OpenHUTB/ros2 的步骤

1. **把文档放进仓库**：将 `docs_musculoskeletal_knee_tracking.md` 复制到仓库
   `docs/musculoskeletal/musculoskeletal_knee_tracking.md`；
   源码放入 `src/mujoco_musculoskeletal_knee_tracking/`。
2. **注册导航**：在仓库 `docs/mkdocs.yml` 与首页 `docs/index.md` 中，于
   「肌肉骨骼人」分类下添加跳转链接（模块名唯一，不与 `elbow` 等重复）。
3. **运行验证（重要）**：在 Ubuntu 上执行
   `mkdocs serve --livereload` 检查页面效果；确保
   `colcon build` + `ros2 launch ... main.launch.py` 可直接运行。
4. **提 PR**：
   - 标题和提交信息说明「为什么修改」，而非仅「改了什么」；
   - `File changed` 内容符合预期、无冲突（有冲突先合并最新代码）；
   - 附运行效果图（`tracking.gif` 用 ScreenToGif 录制，PNG<3MB、GIF<10MB）；
   - README 写明运行环境与步骤；
   - 页面有红色提示(冲突)时先同步再提交。
5. **找一位同学在另一台机器测试并同意**后即可合并。
6. **声明大模型使用**：本模块开发使用了大模型辅助，已在提交中声明并对内容负责。

## 三、在 Ubuntu 上完整跑一遍的命令

```bash
pip install mujoco numpy matplotlib

# 离线演示(无需 ROS2, 生成 GIF)
python3 simulate_tracking.py

# ROS2 运行
cd ~/ros2_ws/src
cp -r <本模块目录> ./mujoco_musculoskeletal_knee_tracking
cd ~/ros2_ws && colcon build --packages-select mujoco_musculoskeletal_knee_tracking
source install/setup.bash
ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py
```

## 四、注意事项

- 文件名全英文、下划线分隔（已符合）。
- 模块名 `mujoco_musculoskeletal_knee_tracking` 具体且不与仓库现有模块重复。
- 若课程对「肌肉骨骼人」分类有额外要求，按老师最新说明调整分类与文档位置。
