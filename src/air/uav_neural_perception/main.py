#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_neural_perception 统一入口。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_neural_perception.perception_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
