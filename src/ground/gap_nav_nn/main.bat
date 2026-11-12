@echo off
REM 统一入口：转发给 scripts\main.py
REM 用法: main.bat --mode expert --save-gif gap.gif
python "%~dp0scripts\main.py" %*
