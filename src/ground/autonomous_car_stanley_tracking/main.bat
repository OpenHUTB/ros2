@echo off
REM 一键入口（Windows）：安装依赖并跑完整离线实验，导出效果图与指标
setlocal
cd /d "%~dp0"

echo [1/3] 安装 Python 依赖（numpy 必需，matplotlib/imageio 用于出图与动图）
python -m pip install --user -r requirements.txt

echo [2/3] 运行离线单元测试（不需要 ROS 与仿真器）
python test\test_stanley.py

echo [3/3] 运行全部场景并导出效果图到 results\
python main.py --mode demo

echo 完成。效果图见 results\ 目录，指标见 results\metrics.csv
endlocal
