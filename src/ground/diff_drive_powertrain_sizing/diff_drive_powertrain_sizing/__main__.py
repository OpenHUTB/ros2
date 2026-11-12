"""`python3 -m diff_drive_powertrain_sizing` 入口（launch 找不到 console script 时的兜底）."""

from .cli import main

if __name__ == '__main__':
    raise SystemExit(main())
