#!/usr/bin/env python3
"""肌肉驱动夹爪统一入口。

    python3 main.py check
    python3 main.py bench --out out
    python3 main.py gif --out gif
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from mujoco_muscle_gripper.cli import main  # noqa: E402

if __name__ == "__main__":
    main()
