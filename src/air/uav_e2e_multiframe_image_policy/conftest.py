"""离线测试导入路径配置：把功能包父目录加入 sys.path。"""

import os
import sys

_PKG_PARENT = os.path.dirname(os.path.abspath(__file__))
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)
