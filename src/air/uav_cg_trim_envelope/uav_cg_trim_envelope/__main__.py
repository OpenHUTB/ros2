"""`python3 -m uav_cg_trim_envelope` 入口（launch 找不到 console script 时的兜底）."""

from .cli import main

if __name__ == '__main__':
    raise SystemExit(main())
