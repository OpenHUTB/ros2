@echo off
rem CARLA 端到端神经网络（图像 → 控制）—— Windows 一键运行脚本
rem
rem   main.bat                                 在线：端到端 CNN 自主驾驶
rem   main.bat --mode collect --host <IP>      行为克隆采集 (图像, 专家转向)
rem   main.bat --mode train --epochs 60        训练 CNN（无需 CARLA）
rem   main.bat --headless --demo --save_dir shots
rem                                            离线取证：合成道路图像训练并导出图
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
set "PYTHONPATH=%SCRIPT_DIR%;%PYTHONPATH%"

echo ==================================================
echo   CARLA 端到端神经网络（图像 -^> 控制）
echo ==================================================

python "%SCRIPT_DIR%main.py" %*
endlocal
