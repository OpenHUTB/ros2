@echo off
REM 统一入口（Windows）：check / run / bench / gif
cd /d "%~dp0"
python main.py %*
