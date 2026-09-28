@echo off
REM carla_manual_control Windows 启动入口
REM 依赖：Python 3.14 + hutb cp314 + pygame-ce
cd /d %~dp0
python main.py --host=localhost --port=2000
pause
