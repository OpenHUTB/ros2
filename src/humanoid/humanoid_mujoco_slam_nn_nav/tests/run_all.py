"""无 ROS 的单元测试集合：python3 tests/run_all.py"""
import os
import sys
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

MODULES = ["test_ik", "test_mapping", "test_planning", "test_nn"]


def main():
    failed = 0
    for name in MODULES:
        try:
            mod = __import__(name)
            if hasattr(mod, "main"):
                mod.main()
            print(f"[PASS] {name}")
        except Exception:
            failed += 1
            print(f"[FAIL] {name}")
            traceback.print_exc()
    print(f"\n{len(MODULES) - failed}/{len(MODULES)} test modules passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
