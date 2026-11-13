import os
import sys

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIBLING_ROOT = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", "uav_mujoco_sim"))

for path in (PACKAGE_ROOT, SIBLING_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)
