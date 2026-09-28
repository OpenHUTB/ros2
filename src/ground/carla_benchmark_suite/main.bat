@echo off
rem CARLA 作业综合整合与性能评价 —— Windows 一键运行脚本
rem
rem   main.bat                              打印帮助
rem   main.bat --list                       列出所有可调度模块
rem   main.bat --benchmark --save_dir shots --out report.json
rem                                         运行基准评测（真实测量，无需 CARLA）
rem   main.bat --target perception -- --headless --demo
rem                                         调度作业二（"--" 之后参数原样透传）
setlocal
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
set "PYTHONPATH=%SCRIPT_DIR%;%PYTHONPATH%"

echo ==================================================
echo   CARLA 作业综合整合与性能评价
echo ==================================================

python "%SCRIPT_DIR%main.py" %*
endlocal
