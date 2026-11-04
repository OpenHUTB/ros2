@echo off
rem MuJoCo 两轮自平衡小车 —— Windows 一键运行脚本
rem
rem   main.bat                     离线批跑 + 导出 CSV/PNG（默认）
rem   main.bat --gui               可视化窗口（需要图形界面）
rem   main.bat --selftest          单元测试
rem   main.bat --lean 0.2 --duration 8 --save_dir shots
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
set "PYTHONPATH=%SCRIPT_DIR%;%PYTHONPATH%"

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

"%PYTHON_BIN%" "%SCRIPT_DIR%main.py" %*
endlocal
