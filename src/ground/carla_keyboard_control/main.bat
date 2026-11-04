@echo off
rem CARLA 0.9.16 地面载具物理仿真与键盘运动控制 —— Windows 一键运行脚本
rem
rem   main.bat                       独立模式（pygame 窗口 + 键盘控制）
rem   main.bat --host 192.168.8.1    指定 CARLA 服务端地址
rem   main.bat --follow              镜头跟随自车（便于录屏）
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

echo ==================================================
echo   CARLA 0.9.16 地面载具物理仿真与键盘运动控制
echo ==================================================

python "%SCRIPT_DIR%main.py" %*
endlocal
