"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../../docs/img/ground/rtk_ambiguity_resolution
"""

import sys

from rtk_ambiguity_resolution.headless import main

if __name__ == "__main__":
    sys.exit(main())
