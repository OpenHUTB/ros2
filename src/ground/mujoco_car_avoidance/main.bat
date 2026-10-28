@echo off
REM mujoco_car_avoidance 一键入口（Windows）
REM 用法： main.bat --demo / main.bat --headless / main.bat --test
REM 无参数时默认跑无窗口模式
setlocal
if "%~1"=="" (
    python "%~dp0main.py" --headless
) else (
    python "%~dp0main.py" %*
)
endlocal
