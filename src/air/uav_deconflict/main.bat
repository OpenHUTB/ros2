@echo off
REM Windows 下源码目录直接运行（不需要 colcon install）
setlocal
set HERE=%~dp0
set PYTHONPATH=%HERE%;%HERE%..\uav_mujoco_sim;%PYTHONPATH%
python "%HERE%main.py" %*
endlocal
