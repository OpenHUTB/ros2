# common —— 三个运动规划模块共用的 Python 工具

本目录只放**三个包都要用的两个文件**，源码树里各存一份，避免同一份代码在
`rrt_path_planning`、`trapezoidal_profile`、`stanley_tracking` 里各贴一遍。

| 文件 | 作用 | 谁在用 |
|---|---|---|
| `simple_plot.py` | 手写 zlib PNG 的绘图小库（`Figure` / `Canvas`），不依赖 matplotlib | 三个包的 `scripts/simulate.py`（离线出图） |
| `sim_node.py` | 运动学自行车模型节点：订阅 `/cmd_vel`，发布 `/odom` | 三个包的 `launch/*.launch.py`（闭环里的仿真器一侧） |

## 为什么不做成一个 ament 包

本仓库的模块都是一个 `src/<栏目>/<模块>/` 独立包，评审要求"包可单独拷走就能构建"。
如果把共用代码做成第四个 ament 包，单独拷一个模块时就得连那个包一起拷，
反而破坏了这个约定。所以这里选的是**源码唯一、构建时各包各自拷一份**：

```cmake
set(MOTION_COMMON "${CMAKE_CURRENT_SOURCE_DIR}/../common")

install(PROGRAMS ${MOTION_COMMON}/sim_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES    ${MOTION_COMMON}/simple_plot.py DESTINATION lib/${PROJECT_NAME})
```

装完之后 `install/<包名>/lib/<包名>/` 里就有这两个文件，与包内脚本同目录，
运行时通过 `sys.path` 直接 `import simple_plot` 即可 —— 见各包
`scripts/simulate.py` 顶部的查找逻辑（先看脚本同目录，再看 `../../common`，
源码树和安装后两种情况都能命中）。

## 单独运行某个包时

`python3 scripts/main.py` 会按 `../../common` 找到这里的 `simple_plot.py`，
不需要安装、不需要设置 `PYTHONPATH`。把某个包目录单独拷走后再运行，
把本目录一起拷到同级位置即可（`<包>/../common`）。
