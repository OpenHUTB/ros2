from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
ARM_PACKAGE = (
    REPO_ROOT
    / 'src'
    / 'musculoskeletal'
    / 'mujoco_musculoskeletal_arm_reach'
)
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(ARM_PACKAGE))
