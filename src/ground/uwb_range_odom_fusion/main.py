"""Module entry point.

Examples
--------
Full offline benchmark with figures::

    python main.py --seeds 10 --out results --figure-dir ../../docs/img/ground/uwb_range_odom_fusion

Quick smoke run without matplotlib::

    python main.py --seeds 1 --duration 10 --no-plots --out /tmp/uwb_smoke
"""

import sys

from uwb_range_odom_fusion.headless import main

if __name__ == "__main__":
    sys.exit(main())
