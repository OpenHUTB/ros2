@echo off
REM Carla Traffic Sign Recognition 启动入口
cd /d %~dp0

REM 检查是否在指定的虚拟环境中
if exist D:\WWW\venv_carla310\Scripts\python.exe (
    echo [INFO] Found venv at D:\WWW\venv_carla310
    D:\WWW\venv_carla310\Scripts\python.exe main.py
) else (
    echo [WARNING] Venv not found at D:\WWW\venv_carla310
    echo [WARNING] Ensure you have activated the correct Python environment!
    python main.py
)
pause
