"""让 pytest 能找到包：把模块父目录加入 sys.path。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
