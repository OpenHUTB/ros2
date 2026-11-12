"""Module entry point: offline experiment without ROS.

    python main.py --out results --figure-dir ../../../docs/img/ground/encoder_quantization_velocity
"""

import sys

from encoder_quantization_velocity.headless import main

if __name__ == "__main__":
    sys.exit(main())
