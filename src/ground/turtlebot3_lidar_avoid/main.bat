@echo off
chcp 65001 >nul
REM ============================================================================
REM TurtleBot3 激光雷达自主避障 —— Windows 端启动脚本
REM ============================================================================
REM
REM 说明：
REM   ROS 2 仿真本体运行在 Linux（Ubuntu + ROS 2 Humble）中。
REM   本脚本供 Windows 宿主机通过 SSH 调用虚拟机 / WSL 内的仿真节点，
REM   与其他模块的入口约定保持一致。
REM
REM 用法：
REM   main.bat               默认 /scan
REM   main.bat /out/scan     指定雷达话题
REM
REM 前置条件：已配置到 ROS 2 主机的 SSH 免密登录；或在 Linux 上直接运行 main.sh
REM ============================================================================

setlocal

set SCAN_TOPIC=%1
if "%SCAN_TOPIC%"=="" set SCAN_TOPIC=/scan

REM ---- 配置区（支持通过环境变量提前设置，或在此处修改）----
if "%ROS_HOST%"=="" set ROS_HOST=YOUR_ROS_HOST_IP
if "%WS_PATH%"=="" set WS_PATH=/home/user/ros2_ws
set PKG_NAME=turtlebot3_lidar_avoid

echo ==================================================
echo   TurtleBot3 激光雷达自主避障 (Windows SSH 入口)
echo ==================================================
echo   雷达话题 : %SCAN_TOPIC%
echo   目标主机 : %ROS_HOST%
echo   工作空间 : %WS_PATH%
echo ==================================================
echo.

if "%ROS_HOST%"=="YOUR_ROS_HOST_IP" (
    echo [提示] 尚未配置目标 ROS 2 主机地址。
    echo 请在命令行预先设置环境变量，例如：
    echo   set ROS_HOST=your_user@your_ip
    echo   main.bat
    echo 或直接在 Linux / WSL 环境下执行 bash main.sh
    exit /b 1
)

where ssh >nul 2>nul
if errorlevel 1 (
    echo [错误] 未找到 ssh 命令，请在 Linux / WSL 环境下直接运行 bash main.sh
    exit /b 1
)

ssh %ROS_HOST% "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; [ -f %WS_PATH%/install/setup.bash ] && source %WS_PATH%/install/setup.bash; ros2 run %PKG_NAME% avoid_node --ros-args -p scan_topic:=%SCAN_TOPIC%"

endlocal
