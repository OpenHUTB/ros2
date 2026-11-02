# carla_traffic_light

CARLA 交通信号灯识别与响应。

车辆接近路口时读取信号灯状态，按「红灯停 / 黄灯看能否停住 / 绿灯行」决策，
用 PI + 前馈的纵向控制器执行，并在 CARLA 里保存运行截图。

详细说明见站点文档：`docs/ground/carla_traffic_light.md`

## 运行环境

开发和验证都在 **Ubuntu 20.04 + Python 3.8** 上完成。

| 项目 | 要求 |
|---|---|
| 操作系统 | **Ubuntu 20.04**（Windows 也能跑，见文末） |
| Python | 3.7 / 3.8（Ubuntu 20.04 自带 3.8） |
| CARLA | 0.9.13 / 0.9.16（离线模式不需要 CARLA） |
| ROS | Noetic（用 launch 启动时需要） |

代码里没有任何平台相关调用，Windows / Ubuntu 通用。

## 安装

### 1. 基础依赖

```bash
sudo apt update
sudo apt install -y python3-pip python3-dev
```

### 2. CARLA 客户端库

> **⚠ 版本坑**：PyPI 上 `carla` 0.9.16 只发布了 **cp310 / cp311 / cp312** 的轮子，
> **没有 Python 3.8 的版本**。Ubuntu 20.04 自带 Python 3.8，直接
> `pip3 install carla==0.9.16` 会报 `No matching distribution found`。
> 0.9.15 倒是有 cp38 轮子，但客户端 0.9.15 连 0.9.16 服务端会
> `std::bad_alloc` 崩溃，版本必须对上。
>
> 所以先准备一个 Python 3.10（用 uv 最省事，不用动系统）：
>
> ```bash
> curl -LsSf https://astral.sh/uv/install.sh | sh
> ~/.local/bin/uv python install 3.10
> ~/.local/bin/uv venv --python 3.10 ~/carla_env
> ~/.local/bin/uv pip install --python ~/carla_env/bin/python carla==0.9.16
> ```
>
> 之后跑真机都用 `~/carla_env/bin/python`。离线仿真和单元测试用系统
> `python3` 就行，不需要 CARLA。

以下是 0.9.13 等支持 Python 3.8 的版本可以用的方式：

**方式 A：用 CARLA 自带的 PythonAPI（推荐，版本一定对得上）**

```bash
# 假设 CARLA 解压在 ~/CARLA_0.9.13
export CARLA_HOME=~/CARLA_0.9.13
export PYTHONPATH=$PYTHONPATH:$CARLA_HOME/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
```

建议把这两行加到 `~/.bashrc` 里，省得每次重开终端都要设。

> 注意把 `0.9.13-py3.7` 换成你实际的文件名。用 `ls $CARLA_HOME/PythonAPI/carla/dist/` 看一下。

**方式 B：pip 安装**

```bash
pip3 install carla==0.9.13
```

### 3. 验证安装

```bash
python3 -c "import carla; print('ok')"
```

能打印 `ok` 就行。

## 运行

### 离线仿真（不需要 CARLA，能出图）

```bash
cd scripts
python3 main.py                       # 默认红灯
python3 main.py --light Yellow        # 黄灯
python3 main.py --all-lights          # 依次跑红/黄/绿
```

会打印判决过程并生成曲线到 `scripts/demo_out/`。

### 连接 CARLA 跑真车

**终端 1**，启动 CARLA 服务器：

```bash
cd ~/CARLA_0.9.13
./CarlaUE4.sh -quality-level=Low -windowed -ResX=1000 -ResY=700
```

没有图形界面的服务器上可以用离屏渲染：

```bash
./CarlaUE4.sh -RenderOffScreen -quality-level=Low
```

等窗口出现（或端口 2000 起来）：

```bash
ss -ltnp | grep 2000
```

**终端 2**，运行模块：

```bash
cd src/ground/carla_traffic_light/scripts
python3 main.py --carla --duration 30
```

截图保存到 `scripts/screenshots/`。

### 用 launch 启动

把模块放进 catkin 工作空间：

```bash
cd ~/catkin_ws/src
ln -s /path/to/ros2/src/ground/carla_traffic_light .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

然后：

```bash
roslaunch carla_traffic_light offline.launch      # 离线
roslaunch carla_traffic_light main.launch         # 连 CARLA
```

## 测试

```bash
python3 -m unittest discover -s tests
```

跑完应显示 `Ran 46 tests ... OK`。

决策逻辑和控制器都是纯 Python，**不需要 CARLA 就能测**。

## 文件

```
carla_traffic_light/
├── package.xml / CMakeLists.txt / setup.py    ROS 包
├── launch/main.launch                         连 CARLA
├── launch/offline.launch                      离线
├── config/tl_params.yaml                      参数
├── scripts/
│   ├── traffic_light_logic.py                 决策逻辑（纯 Python，可测）
│   ├── longitudinal.py                        PI + 前馈纵向控制（纯 Python）
│   ├── carla_runner.py                        CARLA 集成
│   ├── simple_plot.py                         PNG 折线图
│   └── main.py                                入口
└── tests/test_logic.py                        46 个测试
```

前两个脚本不依赖 CARLA，可以单独测试；`carla_runner.py` 只在跑真车时用到。

## Windows 上运行（可选）

Windows 也能跑，把上面两处命令换掉即可：

```powershell
cd D:\CARLA\CARLA_0.9.16
.\CarlaUE4.exe -quality-level=Low -windowed -ResX=1000 -ResY=700
```

客户端库用 `pip install carla==0.9.16`（需要 Python 3.12 以下版本对应）。
其余 `python3 main.py ...` 命令完全一样。

## 已知限制

* 只做纵向为主，转向仅用纯跟踪保持车道，不做变道和绕行
* 依赖 CARLA 的 `get_stop_waypoints()`。某张地图上信号灯如果没有停止线，
  该灯会被跳过
* 单辆车，不考虑其他交通参与者
