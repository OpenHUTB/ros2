@echo off
rem MuJoCo AEB —— Windows 一键运行脚本
rem
rem   main.bat                     离线批跑 + 导出 CSV/PNG（默认）
rem   main.bat --gui               可视化窗口（需要图形界面）
rem   main.bat --selftest          单元测试
rem   main.bat --v-ego 12 --ttc 6  改工况与阈值
setlocal
set "PACKAGE_ROOT=%~dp0"
cd /d "%PACKAGE_ROOT%"
set "PYTHONPATH=%PACKAGE_ROOT%;%PACKAGE_ROOT%\scripts;%PYTHONPATH%"

set "MJ_CORE_GLFW=software"

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
