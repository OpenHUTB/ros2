@echo off
REM R2D2 航点巡航仿真启动脚本（Windows）
REM 用法: main.bat

echo 启动 R2D2 小车航点巡航仿真...
python "%~dp0src\r2d2_waypoint_cruise\cruise.py"
pause
