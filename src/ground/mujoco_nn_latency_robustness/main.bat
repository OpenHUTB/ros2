@echo off
REM 训练 + 实验 + 出图出 GIF（Windows）
cd /d "%~dp0"
pip install -q -r requirements.txt
set MPLBACKEND=Agg
python main.py --demo %*
