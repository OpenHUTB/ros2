"""包根入口：让 python main.py 与 ros2 run 走同一份代码。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from uav_bc_bench.main import main  # noqa: E402

if __name__ == '__main__':
    raise SystemExit(main())
