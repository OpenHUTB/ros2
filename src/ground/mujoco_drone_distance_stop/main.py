"""保留原运行入口。 Preserve the original entry point."""
from pathlib import Path
import runpy
import sys

module = Path(__file__).resolve().parents[2] / "air" / "mujoco_uav_range_neural_braking"
sys.path.insert(0, str(module))
if __name__ == "__main__":
    runpy.run_path(str(module / "main.py"), run_name="__main__")
