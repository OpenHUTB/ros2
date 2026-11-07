#!/usr/bin/env python3
"""ROS entry point for drl_drone_path_planning.

``catkin_install_python`` wraps this file, so ``__file__`` may point at the
generated wrapper.  The launch file exports ``DRL_NAV_SCRIPT_DIR`` with the
real source directory; importing from that directory guarantees the node sees
``common.py``/``policy.py``/``baseline.py`` instead of the wrappers.

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import os
import sys


def _prepare_path() -> None:
    """Put the real script directory first on ``sys.path``."""
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (os.environ.get("DRL_NAV_SCRIPT_DIR"), here):
        if candidate and candidate not in sys.path:
            sys.path.insert(0, candidate)


def main() -> None:
    """Start the ROS control node."""
    _prepare_path()
    from drl_nav_node import main as run_node

    run_node()


if __name__ == "__main__":
    main()
