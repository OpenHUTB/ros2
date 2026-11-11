# ros1_env_check —— ROS 1 环境自检

跑课程模块时报的错，很多其实不是代码问题，而是**环境没配好**：
终端里混了 ROS 2、忘了 `source`、缺 `python3-numpy`、脚本带 BOM 或 CRLF、没有可执行位。
这些错误的提示信息往往指向别处，容易误判成"模块有 bug"。

本模块把这类检查集中到一处，一次性打印**什么没问题、什么有问题、怎么修**。
只依赖 Python 标准库，**不需要 ROS 也能运行**——在没装 ROS 的机器上它同样能给出结论，而不是自己先崩掉。

## 快速开始

```bash
cd src/chap2/ros1_env_check

./main.sh                            # 检查当前终端与环境
python3 check_env.py --scan .        # 额外扫描目录下的 BOM / CRLF / 可执行位
python3 check_env.py --scan . --scan-only   # 只按文件检查计退出码（CI 做文档 lint 用）
python3 check_env.py --strict        # 把警告也算失败（适合放进 CI）
python3 check_env.py --quiet         # 只打印问题
```

退出码：`0` = 通过，`1` = 有 FAIL（`--strict` 时含 WARN），`2` = 参数错误。

**`--scan-only` 为什么要单独存在**：不加它时，退出码把环境检查也算进去。
在**没装 ROS 的 CI job** 里跑 `--scan docs` 会一直红（`ROS_DISTRO 未设置`、
`import rospy 失败` 与文件无关）。加上 `--scan-only` 后只跑文件卫生、只按它计退出码：

```bash
# 干净目录、没 source ROS：仍然退出 0
python3 check_env.py --scan docs/ --scan-only
```

## 它检查什么

| 分组 | 检查项 |
|---|---|
| ROS 环境 | `ROS_DISTRO` / `ROS_VERSION` 是否 ROS 1；`/opt/ros` 下装了哪些发行版；**ROS 1 与 ROS 2 是否共存、当前终端 source 的是哪一个**；`AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH` 是否残留 ROS 2 路径；`rosversion` 是否可用；`ROS_MASTER_URI` |
| Python 与依赖 | Python 版本 ≥ 3.6；`rospy`、`numpy` 必装；`yaml`、`matplotlib`、`catkin_pkg`、`tf` 可选 |
| 工作空间 | 向上查找 **带 catkin 标记**（`.catkin_workspace` / `devel` / `build`）且含 `src/` 的工作空间；`build/`、`devel/` 是否存在；`devel` 的 Python 路径是否已进入 `sys.path`（等价于"有没有 `source devel/setup.bash`"）；`src/` 下的包数量 |
| 文件卫生（`--scan`） | UTF-8 **BOM**、**CRLF** 行尾、`.sh` **缺可执行位** |


## 为什么特别关注「ROS 1 与 ROS 2 共存」

课程环境常见 Ubuntu 20.04 + **Noetic** + **Foxy** 双版本共存。这时真正的问题不是"没装 ROS"，
而是**当前终端 source 了哪一个**。若先 source 了 Foxy 再 `catkin_make`，会出现：

```
ROS_DISTRO was set to 'foxy' before. Please make sure that the environment does not mix paths
```

随后 `catkin_make` 行为异常、`rospy` 导入失败、节点启动即退出。正确做法是**关掉终端重开**，
只在干净终端里：

```bash
source /opt/ros/noetic/setup.bash      # 不要再 source foxy 的 setup.bash
```

本脚本会把 `/opt/ros` 下的已安装版本列出来，并明确指出当前终端处于哪一版。

## 文件结构

```
src/chap2/ros1_env_check/
├── check_env.py            # 全部检查逻辑（纯标准库，可单独运行）
├── main.sh                 # 一键包装：定位自身目录后调用 check_env.py
├── package.xml             # catkin 包声明（只为与同章节其它模块一致，不依赖 ROS 运行）
├── CMakeLists.txt          # 把两个入口安装到 devel/lib/ros1_env_check
├── test/
│   └── test_check_env.py   # 18 项回归测试（subprocess 调命令行，断言退出码与输出）
└── README.md
```

**没有 `launch/`，这是刻意的**：本模块是独立 CLI 工具，不是 ROS 节点
（它甚至要在"ROS 还没 source 好"的环境里运行才最有价值）。
用 `roslaunch` 去启动一个跑完就退出的诊断脚本并不合适，
所以入口就是上表的 `./main.sh` 与 `python3 check_env.py`。

## 测试

```bash
cd src/chap2/ros1_env_check
python3 -m pytest test/ -q        # 用 pytest 跑
python3 test/test_check_env.py    # 不装 pytest 也能跑（18 项断言）
```

测试用 `subprocess` 直接调命令行，断言的是**退出码与输出文字**——
这类脚本最容易回归的正是"该失败的时候退出码是不是 1"。

## 已知限制

1. **可执行位检查只在 Linux/macOS 生效**：Windows 没有 POSIX 权限位，
   `os.access(path, X_OK)` 在 Windows 上只检查文件是否存在，会产生假阴性，因此显式跳过。
2. **只在 `/opt/ros` 下查找已安装发行版**：通过源码编译或装在自定义路径的 ROS 不会被列出，
   但 `ROS_DISTRO` 的判断仍然有效。
3. **不代替真正的编译验证**：环境自检通过不代表模块能编译；
   `catkin_make` 与 `roslaunch` 仍需单独执行。
4. **`--scan` 只认常见文本扩展名**（`.py` `.sh` `.md` `.launch` `.xml` `.yaml` `.yml`
   `.cpp` `.h`），`.urdf` `.rviz` `.pgm` 不在范围内，需要时扩展 `SCAN_EXTS`。
5. **`core.autocrlf=true` 时跳过 CRLF 检查**：此时工作区本来就是 CRLF（提交后转 LF），
   逐个报 CRLF 全是假阳性，所以说明原因后跳过，而不是制造噪声。

## 三处容易给出错误结论的判定

* **`--scan` 的路径不存在**：早期版本 `os.walk` 什么都不产出，三项检查全 OK ——
  CI 里路径写错也全绿。现在直接 FAIL 并退出码 1；传文件而非目录、目录里没有可扫文件也会明确提示。
* **仓库源码树 ≠ catkin 工作空间**：仓库根目录也有 `src/`，只按 `src/` 判断会建议你在此 `catkin_make`，
  而那会生成提交指引不允许的 `build/`、`devel/`。现在要求同时存在
  `.catkin_workspace` / `devel/` / `build/` 之一，源码树里改为提示"软链到独立工作空间"。
* **`core.autocrlf=true`**：见上面第 5 条。
