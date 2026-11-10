@echo off
chcp 65001 >nul
rem -*- coding: utf-8 -*-
rem humanoid_embodied_control —— Windows 一键运行入口（main.bat）
rem
rem 用法：
rem   main.bat --mode sim --seconds 30
rem   main.bat --mode offline
rem   main.bat --help
rem
rem 说明：ROS 1 Noetic 自带 python3.8，而 MuJoCo 3.x 需要 3.9+；
rem 本脚本按以下优先级挑选解释器：
rem   1. 环境变量 HUMANoid_PYTHON
rem   2. python / python3（PATH 中第一个）

setlocal
cd /d "%~dp0"

set "HUMANOID_PY=%HUMANOID_PYTHON%"
if "%HUMANOID_PY%"=="" (
  where python >nul 2>nul && set "HUMANOID_PY=python"
)
if "%HUMANOID_PY%"=="" (
  where python3 >nul 2>nul && set "HUMANOID_PY=python3"
)
if "%HUMANOID_PY%"=="" (
  echo [humanoid_embodied_control] 未找到 python
  exit /b 1
)

echo [humanoid_embodied_control] using %HUMANOID_PY%
%HUMANOID_PY% -c "import mujoco" >nul 2>nul
if errorlevel 1 (
  echo [humanoid_embodied_control] mujoco 未安装，尝试安装...
  %HUMANOID_PY% -m pip install -q mujoco numpy scipy
)

%HUMANOID_PY% main.py %*
exit /b %errorlevel%
