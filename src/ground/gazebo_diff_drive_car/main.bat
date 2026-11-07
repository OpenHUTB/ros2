@echo off
REM ============================================================================
REM Gazebo 差速驱动小车 —— Windows 端启动脚本
REM ============================================================================
REM
REM 说明：ROS 2 与 Gazebo 运行在 Linux（Ubuntu 22.04 + ROS 2 Humble）中。
REM       本脚本供 Windows 宿主机通过 SSH 调用虚拟机内的仿真，
REM       与其他模块的入口约定保持一致。
REM
REM 用法：main.bat [teleop^|laser^|camera^|idle]
REM ============================================================================

setlocal

set MODE=%1
if "%MODE%"=="" set MODE=teleop

REM ---- 配置区 ----
set ROS_HOST=user@192.168.93.129
set WS_PATH=/home/user/ros2_ws

echo ==================================================
echo   Gazebo 差速驱动小车
echo ==================================================
echo   运行模式 : %MODE%
echo   目标主机 : %ROS_HOST%
echo ==================================================
echo.

where ssh >nul 2>nul
if errorlevel 1 (
    echo [错误] 未找到 ssh 命令，请在 Linux 上直接运行 main.sh
    exit /b 1
)

ssh %ROS_HOST% "source /opt/ros/humble/setup.bash && source %WS_PATH%/install/setup.bash && ros2 launch gazebo_diff_drive_car main.launch.py mode:=%MODE%"

endlocal
