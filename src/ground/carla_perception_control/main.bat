@echo off
rem CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）—— Windows 一键运行脚本
rem
rem   main.bat                              在线：连 CARLA，NN 感知 + 轨迹跟踪
rem   main.bat --mode train                 离线训练两个神经网络（无需 CARLA）
rem   main.bat --headless --demo --save_dir shots
rem                                         离线取证：导出曲线图（无 CARLA 也能跑）
rem   main.bat --host 192.168.8.1           指定 CARLA 服务端地址
rem   main.bat --host 192.168.8.1 --follow  第三人称跟随镜头（观察/录屏用）
rem
rem 连不上 CARLA 时先做连通性诊断：
rem   python check_connection.py <宿主机IP> 2000 Town05
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
set "PYTHONPATH=%SCRIPT_DIR%;%PYTHONPATH%"

echo ==================================================
echo   CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）
echo ==================================================

python "%SCRIPT_DIR%main.py" %*
endlocal
