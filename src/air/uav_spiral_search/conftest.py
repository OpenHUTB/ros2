"""让单元测试不依赖 colcon 安装即可从任意目录运行。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
