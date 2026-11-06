@echo off
rem Windows 启动脚本：等价于在该目录下执行 python main.py %*
cd /d "%~dp0"
python main.py %*
