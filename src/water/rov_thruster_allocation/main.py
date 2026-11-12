"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../../docs/img/water/rov_thruster_allocation
"""

import sys

from rov_thruster_allocation.headless import main

if __name__ == "__main__":
    sys.exit(main())
