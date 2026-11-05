@echo off
rem CARLA 建图 + 导航（神经网络规划）—— Windows 一键运行脚本
rem
rem   main.bat                              在线：LiDAR 建图 + NN 规划导航
rem   main.bat --mode train                 离线训练规划神经网络（无需 CARLA）
rem   main.bat --headless --demo --save_dir shots
rem                                         离线取证：合成环境建图+导航并导出地图
rem   main.bat --goal 20,8                  指定导航目标
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
set "PYTHONPATH=%SCRIPT_DIR%;%PYTHONPATH%"

echo ==================================================
echo   CARLA 建图 + 导航（神经网络规划）
echo ==================================================

python "%SCRIPT_DIR%main.py" %*
endlocal
