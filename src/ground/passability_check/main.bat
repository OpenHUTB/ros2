@echo off
REM passability_check 一键入口（Windows）
REM 用法： main.bat                         跑默认车型 + 全部场景
REM        main.bat --vehicle suv            换车型
REM        main.bat --vehicle all --matrix   看车型 × 场景对比表
REM        main.bat --test                   自检（不依赖 ROS）
setlocal
if "%~1"=="" (
    python "%~dp0main.py"
) else (
    python "%~dp0main.py" %*
)
endlocal
