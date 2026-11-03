@echo off
REM mujoco_car_rollover_guard 一键入口（Windows）
REM 用法： main.bat --collect / --train / --eval / --demo / --test
REM 无参数时默认跑三组策略对比评测
setlocal
if "%~1"=="" (
    python "%~dp0main.py" --eval
) else (
    python "%~dp0main.py" %*
)
endlocal
