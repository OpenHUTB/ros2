@echo off
REM ============================================================================
REM main.bat —— OpenHUTB 无人车（Gazebo）Windows 端启动脚本
REM ============================================================================
REM
REM 说明：
REM   Gazebo 仿真本体运行在 Linux（Ubuntu 20.04 + ROS Noetic）环境中。
REM   本脚本用于 Windows 宿主机在虚拟机 / WSL 场景下调用 Linux 侧的仿真实例，
REM   与仓库内其他模块（如 carla_keyboard_control）保持一致的入口约定。
REM
REM 用法：
REM     main.bat                  默认键盘遥控模式
REM     main.bat laser            激光雷达避障
REM     main.bat camera           深度相机避障
REM     main.bat idle             仅加载仿真环境
REM
REM 前置条件（三选一）：
REM   1) 已有可用的 WSL 环境并安装 ROS Noetic
REM   2) 已配置 SSH 免密登录到运行 ROS 的虚拟机
REM   3) 直接在 Linux 机器上运行 main.sh（推荐）
REM
REM ============================================================================

setlocal

REM ---------------------------- 参数 ----------------------------------------
set MODE=%1
if "%MODE%"=="" set MODE=teleop

REM ---------------------------- 配置区（按需修改） ---------------------------
REM ROS 运行环境。二选一，取消相应注释。
set ROS_HOST=user@192.168.93.129
REM set ROS_HOST=wsl

REM 本项目在 ROS 工作区中的包名
set PKG_NAME=gazebo_diff_drive_car

REM Linux 侧 catkin 工作区路径
set WS_PATH=/home/user/Documents/mw_catkin_ws

REM ---------------------------- 执行 ----------------------------------------
echo ==============================================================
echo   OpenHUTB 无人车 —— Gazebo 差速驱动仿真
echo ==============================================================
echo   运行模式 : %MODE%
echo   目标主机 : %ROS_HOST%
echo ==============================================================
echo.

where ssh >nul 2>nul
if errorlevel 1 (
    echo [错误] 未找到 ssh 命令。
    echo        请安装 OpenSSH 客户端，或直接在 Linux 上运行 main.sh。
    exit /b 1
)

ssh %ROS_HOST% "cd %WS_PATH% && source /opt/ros/noetic/setup.bash && source devel/setup.bash && export SVGA_VGPU10=0 && roslaunch %PKG_NAME% main.launch mode:=%MODE%"

endlocal
