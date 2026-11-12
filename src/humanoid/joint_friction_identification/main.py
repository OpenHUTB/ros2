"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../docs/img/humanoid/joint_friction_identification
"""

import sys

from joint_friction_identification.headless import main

if __name__ == "__main__":
    sys.exit(main())
