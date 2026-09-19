# car_keyboard_teleop（仿真车选车与键盘遥控器）

面向 OpenHUTB / AirSim 地面车辆模拟器的「选车 + 键盘驾驶」工具：

1. `main.py` 连接模拟器，实时列出全部可驾驶车型（支持中文车名与关键词搜索），在终端选车；
2. 选定后自动打开第三人称驾驶窗口 `car_manual_control.py`，用键盘开车；
3. 可设置「备选车池」，驾驶中按 `Backspace` 在多辆车之间轮换。

## 运行环境

* 操作系统：Windows 10/11（Linux 同样支持，模拟器本体需自行准备）
* Python：3.8 及以上（开发验证使用 3.11）
* 模拟器：OpenHUTB 预编译地面车辆模拟器（2.9.16，RPC 端口 `2000`）
* Python 依赖：
  * `hutb`：提供 `carla` 模块，**必须与模拟器版本一致**。PyPI 上的版本较旧，请安装模拟器自带的离线 wheel：

    ```shell
    pip install <模拟器安装目录>/PythonAPI/carla/dist/hutb-2.9.16-cp3xx-*-win_amd64.whl
    ```

    （`cp3xx` 选择与本机 Python 版本一致的文件）
  * `pygame`、`numpy`：`pip install -r src/requirements.txt`

## 运行步骤

```shell
# 1. 拉取仓库并进入仓库根目录
git clone https://github.com/OpenHUTB/ros2.git
cd ros2

# 2. 安装依赖
pip install -r src/requirements.txt
pip install <模拟器安装目录>/PythonAPI/carla/dist/hutb-2.9.16-cp3xx-*-win_amd64.whl

# 3. 先启动模拟器（运行 CarlaUE4.exe，等待城市画面出现）

# 4. 运行本模块（入口为 main.py）
python src/ground/car_keyboard_teleop/main.py
```

按菜单提示输入编号或关键词选车；也可以不进菜单直接点名：

```shell
python src/ground/car_keyboard_teleop/main.py -v seal
python src/ground/car_keyboard_teleop/main.py -v mustang --pool cybertruck,model3 --dry-run
```

模拟器未运行时，若能通过 `CARLA_ROOT` 环境变量或上级目录找到安装位置，脚本会尝试自动启动；
找不到则会提示先手动启动模拟器，也可以用 `--simulator` 显式指定可执行文件路径。

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--host` | `127.0.0.1` | 模拟器地址 |
| `--port` | `2000` | CARLA RPC 端口 |
| `--res` | `1280x720` | 驾驶窗口分辨率 |
| `-v, --vehicle` | 空 | 完整蓝图 id、英文片段或中文名 |
| `--pool` | 空 | 备选车池，逗号分隔，`Backspace` 轮换 |
| `--simulator` | 空 | 模拟器可执行文件路径 |
| `--list` | 关 | 只打印车型清单 |
| `--refresh` | 关 | 忽略缓存，强制重新拉取车型 |
| `--no-autostart` | 关 | 不自动启动模拟器 |
| `--dry-run` | 关 | 只选车、打印启动命令，不打开驾驶窗口 |

驾驶键位与完整说明见文档页面：
[仿真车选车与键盘遥控器](../../../docs/ground/car_keyboard_teleop.md)。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `main.py` | 模块入口：连接模拟器、列出/选择车型、启动驾驶窗口 |
| `car_manual_control.py` | 第三人称键盘驾驶窗口（基于 CARLA 0.9.x `manual_control.py` 适配，新增 `--vehicle` 与 `--vehicle-pool`，保留上游 MIT 许可头） |

> 运行期生成的车型缓存与上次选择保存在系统临时目录 `%TEMP%/car_keyboard_teleop/`（Linux 为 `/tmp/car_keyboard_teleop/`），不会写入源码目录。

## 声明

本模块代码与文档部分内容由大模型辅助生成（路径仓库化重构、文档整理等），所有代码均已在本机连接真实模拟器完成运行验证，提交者对全部提交内容负全部责任。
