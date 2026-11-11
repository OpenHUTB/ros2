"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../docs/img/ground/imu_allan_noise_identification
"""

import sys

from imu_allan_noise_identification.headless import main

if __name__ == "__main__":
    sys.exit(main())
