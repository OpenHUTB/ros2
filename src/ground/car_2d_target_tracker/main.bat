@echo off
rem 2D 小车目标追踪 —— Windows 一键运行脚本
rem
rem   main.bat                         离线批跑 + 导出 CSV/PNG（默认）
rem   main.bat --gui                   打开 pygame 窗口（需要 pygame + 图形界面）
rem   main.bat --selftest              单元测试
rem   main.bat --sim-time 60 --seed 7  改时长与随机种子
setlocal
set "PACKAGE_ROOT=%~dp0"
cd /d "%PACKAGE_ROOT%"
set "PYTHONPATH=%PACKAGE_ROOT%;%PYTHONPATH%"

set "PYTHON_BIN=python"
where python >nul 2>nul
if errorlevel 1 (
  where py >nul 2>nul
  if errorlevel 1 (
    echo [错误] 找不到 python，请先安装 Python 3.10+ 并加入 PATH
    exit /b 1
  )
  set "PYTHON_BIN=py"
)

"%PYTHON_BIN%" "%PACKAGE_ROOT%main.py" %*
endlocal
