@echo off
REM 一键跑通：训练 -> 三模式对照 -> 漂移扫描 -> 出图出 GIF
cd /d "%~dp0"
python main.py --demo %*
