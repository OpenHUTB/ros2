@echo off
REM 统一入口（Windows 原生）：按子命令运行各模块。
REM 用法: scripts\main.bat keyboard ^| avoid ^| track ^| explore ^| collect ^| train ^| e2e
setlocal
cd /d "%~dp0\.."

set MODE=%1
if "%MODE%"=="" goto usage

if "%MODE%"=="keyboard" ( python -m modules.task1_keyboard.main %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )
if "%MODE%"=="avoid"    ( python -m modules.task2_perception_control.main --mode avoid %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )
if "%MODE%"=="track"    ( python -m modules.task2_perception_control.main --mode track %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )
if "%MODE%"=="explore"  ( python -m modules.task3_slam_nav.main --mode explore %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )
if "%MODE%"=="collect"  ( python -m modules.task4_e2e.collect_data %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )
if "%MODE%"=="train"    ( python -m modules.task4_e2e.train %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )
if "%MODE%"=="e2e"      ( python -m modules.task4_e2e.main %2 %3 %4 %5 %6 %7 %8 %9 & goto :eof )

:usage
echo 可用命令: keyboard ^| avoid ^| track ^| explore ^| collect ^| train ^| e2e
