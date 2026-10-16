#!/usr/bin/env python3
# ROS node wrapper -> modules.task1_keyboard.main
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from modules.task1_keyboard.main import main
if __name__ == "__main__":
    main()
