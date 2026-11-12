"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../docs/img/ground/mag_hard_soft_iron_calibration
"""

import sys

from mag_hard_soft_iron_calibration.headless import main

if __name__ == "__main__":
    sys.exit(main())
