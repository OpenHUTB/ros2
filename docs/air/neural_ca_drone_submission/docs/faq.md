# 常见问题

## 运行

**Q: 报 `ModuleNotFoundError: No module named 'ca_drone'`**

在仓库根目录执行 `pip install -e .`。若用 conda，确认 `pip` 与 `python`
来自同一环境（`which pip` / `which python`）。

**Q: `AttributeError: module 'numpy' has no attribute 'float'`**

旧代码使用了 NumPy 1.x 的别名。本项目已全部改为 `float64` / `np.float64`，
支持 NumPy 1.20–2.x；若仍报错，说明装的是第三方旧包。

**Q: Pillow 报 `rounded_rectangle` 不存在**

Pillow < 8.2 没有该 API。本项目已用 `_rrect()` 做了运行时兼容，
若仍报错请确认没有调用旧版 `render/scene.py`。

**Q: 没有图形界面能跑吗？**

可以。所有渲染都用 Pillow 在内存里完成，`main.py` 不打开窗口，只写文件。
`--module demo` 才会弹出实时窗口。

**Q: 为什么没有用 PyTorch？**

课程环境不保证有 GPU，PyTorch 的 CPU 版安装包也远大于整个项目。
本项目用 700 行 NumPy 实现反向模式自动微分，四个网络都在 CPU 上训练，
反而让“公式—代码”的对应关系更清楚。代价是卷积较慢（约 1.2 GFLOP/s）。

## 结果

**Q: e2e 为什么只在两个场景成功？**

见[实验与结果](results.md)。简单说：单帧深度 + 目标向量的反应式策略没有记忆，
在长走廊里会在门口来回试探，在目标附近会盘旋；它通过了 `dynamic` 与从未训练过的
`unseen`，但在 `sparse` / `dense` / `corridor` **没有碰撞却超时**。完成率 40%
是如实记录的负结果，不是调参能轻易解决的。

**Q: 为什么 `full` 的规划耗时是 `expert` 的两倍多？**

因为 `full` 每次重规划都要先跑一遍 `NeuralHeuristic` 生成 104×88 的
cost-to-go 场（约 10 ms），再做 A\*；`expert` 用欧氏距离，几乎免费。
在 10 Hz 下 15.3 ms 仍只占单核的 15%。

**Q: 最小间隙为什么不是越大越好？**

间隙大意味着保守、绕远。`control` 的间隙最大（1.92 m）但路径也最长；
`full` 的 1.41 m 是在“安全”与“效率”之间的折中。

## 图表与文档

**Q: matplotlib 中文显示成方框**

本项目图表全部用英文标签，避免字体依赖。文档正文是中文，由 mkdocs 渲染，
不经过 matplotlib。

**Q: GIF 超过 20 MB**

降低 `--render-every` 的抽帧密度（增大数值）、把 `max_width` 调到 560、
`colors` 调到 48。当前 640×400 / 64 色 / 109 帧为 8.93 MB。

**Q: `mkdocs build --strict` 报警告**

严格模式会把“文档中引用了不存在的锚点”当错误。先跑 `mkdocs build`
看具体行号；本仓库的 `mkdocs.yml` 已开启 `strict: false` 的兼容模式，
CI 中使用 `--strict` 时请确保新增页面已加入 nav。

## ROS

**Q: `catkin_make` 找不到 Python 3**

Noetic 默认 Python 3；若同时装了 Anaconda，先 `conda deactivate`。

**Q: `ros2 topic echo` 报 `!rclpy.ok()`**

本机自制 Humble 镜像的 daemon 缺陷，与本包无关。用
`ros2_ws/src/neural_ca_drone_ros2/scripts/probe.py` 验证话题。

**Q: 虚拟机里 `vmrun` 报 `unable to start`**

`vmrun` 通过命名管道与 VMware 通信，某些沙箱/权限模式下会被拒绝；
在普通 PowerShell（非受限模式）中运行即可。
