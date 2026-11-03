#!/usr/bin/env python3
"""Module entry: start the LiDAR ground filter node."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_filter_node import main  # noqa: E402

if __name__ == "__main__":
    main()
