"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../../docs/img/air/uav_wind_estimation
"""

import sys

from uav_wind_estimation.headless import main

if __name__ == "__main__":
    sys.exit(main())
