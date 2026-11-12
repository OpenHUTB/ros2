#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自检：CPG 基线、采样、MLP 形状。直接运行即可。"""
import subprocess, sys
from pathlib import Path

here = Path(__file__).resolve().parent.parent
sys.exit(subprocess.call([sys.executable, str(here / "main.py"), "--test"]))