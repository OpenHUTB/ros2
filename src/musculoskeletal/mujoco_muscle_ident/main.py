#!/usr/bin/env python3
"""肌肉参数辨识统一入口。"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from mujoco_muscle_ident.cli import main  # noqa: E402

if __name__ == "__main__":
    main()
