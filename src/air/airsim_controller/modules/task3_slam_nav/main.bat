@echo off
REM task3_slam_nav 一键启动脚本（Windows）
chcp 65001 >nul
cd /d %~dp0
echo === task3_slam_nav 启动 ===
python main.py %*
pause
