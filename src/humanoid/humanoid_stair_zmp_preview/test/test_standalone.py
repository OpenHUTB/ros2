#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自检：ZMP 闭环 / 前视有效性 / IK 往返 / 上下台阶升幅。

直接运行：python3 test/test_standalone.py
"""
import subprocess, sys
from pathlib import Path

here = Path(__file__).resolve().parent.parent
sys.exit(subprocess.call([sys.executable, str(here / "main.py"), "--test"]))