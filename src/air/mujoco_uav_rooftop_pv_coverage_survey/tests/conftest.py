"""Test import path setup / 测试导入路径设置。"""

from pathlib import Path
import sys


MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT))
