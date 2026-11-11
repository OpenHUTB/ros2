@echo off
REM 在 Windows 上的启动脚本（等价于 python main.py）
cd /d %~dp0
python main.py %*
