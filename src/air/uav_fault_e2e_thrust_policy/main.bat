@echo off
rem 源码树里直接跑（Windows，不需要 ROS）
setlocal
if "%~1"=="" (
  python "%~dp0main.py" --selftest
) else (
  python "%~dp0main.py" %*
)
endlocal
