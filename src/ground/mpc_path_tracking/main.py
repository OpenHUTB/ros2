"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../docs/img/ground/mpc_path_tracking
"""

import sys

from mpc_path_tracking.headless import main

if __name__ == "__main__":
    sys.exit(main())
