"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../docs/img/ground/vio_time_offset_estimation
"""

import sys

from vio_time_offset_estimation.headless import main

if __name__ == "__main__":
    sys.exit(main())
