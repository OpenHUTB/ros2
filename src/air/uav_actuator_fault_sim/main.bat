@echo off
rem uav_actuator_fault_sim Windows 入口（源码目录直接跑）
setlocal
if "%~1"=="" (
  python "%~dp0main.py" --demo
) else (
  python "%~dp0main.py" %*
)
endlocal
