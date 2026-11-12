# 真实环境验证（Ubuntu 虚拟机）

!!! success "结论先行"
    本项目在 **Ubuntu 20.04.6 + Python 3.8.10 + Pillow 7.0.0** 的 VMware 虚拟机中
    完成了：主程序运行、GIF 生成、ROS 1 `catkin_make` + `roslaunch`、
    ROS 2 `colcon build` + `ros2 launch` 全流程验证。

## 1. 为什么需要虚拟机

开发机是 Windows，直接在本机运行只能证明“代码能跑”，不能证明
“在 Ubuntu 20.04 / ROS Noetic 上能跑”。作业明确要求后者，因此用
VMware 启动了一台预装 ROS Noetic + Humble + Gazebo 11 的 Ubuntu 20.04。

## 2. 环境信息

| 项 | 值 |
| --- | --- |
| 操作系统 | Ubuntu 20.04.6 LTS |
| 内核 | 5.15（HWE） |
| CPU / 内存 | 4 vCPU / 8 GB |
| Python | 3.8.10 |
| numpy | 1.24.4 |
| Pillow | **7.0.0** |
| matplotlib | 3.1.2 |
| ROS | Noetic + Humble（`/opt/ros` 并存，默认 Humble） |
| Gazebo | 11.13.0 |

## 3. 部署步骤

### 3.1 打包（宿主机）

```bash
python tools/package.py            # 生成 dist/neural_ca_drone.zip
```

包内排除 `data/*.npz`、`runs/`、`weights_test/`、`debug_*.png`，
只保留源码、权重、文档与启动脚本，约 0.9 MB。

### 3.2 传入虚拟机

```bash
vmrun -T ws -gu user -gp **** copyFileFromHostToGuest \
  "<vmx>" dist/neural_ca_drone.zip /home/user/ca/neural_ca_drone.zip
vmrun -T ws -gu user -gp **** runProgramInGuest "<vmx>" \
  /bin/bash -lc "cd /home/user/ca && unzip -oq neural_ca_drone.zip"
```

### 3.3 运行

```bash
cd /home/user/ca/neural_ca_drone
python3 main.py --module sim --scene sparse
python3 main.py --module full --scene sparse --gif --render-every 4
```

## 4. 实测结果

| 命令 | 结果 |
| --- | --- |
| `main.py --module sim` | exit 0，生成 `frame_sparse.png` |
| `main.py --module full --gif` | exit 0，435 步 / 43.5 s / 51.91 m / 1.294 m / 44 次重规划 |
| GIF 体积 | 8.93 MiB（109 帧，640×400） |
| `catkin_make` | exit 0 |
| `roslaunch ... module:=full` | 两节点启动，`/cmd_vel` 与 `/ca/status` 有数据 |
| `colcon build` | 3.52 s，exit 0 |
| `ros2 launch ... module:=full` | 两节点启动，`probe.py` 收到命令与状态 |
| 深度话题频率 | 10.03 Hz |
| `main.py --module full --scene unseen` | exit 0，456 步成功 / 最小间隙 1.389 m / 46 次重规划 / 37.2 s |
| `main.py --module e2e --scene unseen` | exit 0，到达目标 / 最小间隙 1.204 m / 28.3 s |

## 5. 在这一过程中修掉的两个真实兼容性问题

### 5.1 Python 3.8 不支持 `X | Y` 类型联合

```python
Obstacle = Box | Cylinder          # Python 3.10+ 才合法
```

在 3.8 上报：

```
TypeError: unsupported operand type(s) for |: 'type' and 'type'
```

改成 `typing.Union[Box, Cylinder]`，同时保留 3.10 的可读性。

### 5.2 Pillow 7.0 没有 `rounded_rectangle`

```python
d.rounded_rectangle(box, radius=6, ...)   # Pillow >= 8.2
```

在 7.0 上报：

```
AttributeError: 'ImageDraw' object has no attribute 'rounded_rectangle'
```

新增 `_rrect()`：运行时检测，有圆角就用，没有就退化为直角矩形。
这样同一份渲染代码在 Pillow 7 / 9 / 11 上都能出图。

## 6. 对文档的启示

- 声明“支持 Ubuntu 20.04”不能只写 `requirements.txt`，必须实际跑；
- 旧版本 Pillow / Python 的差异集中在**类型语法**与**绘图 API**，
  渲染与数值代码本身兼容性很好；
- 虚拟机没有 GPU，能跑通就说明算法确实不依赖 CUDA。
