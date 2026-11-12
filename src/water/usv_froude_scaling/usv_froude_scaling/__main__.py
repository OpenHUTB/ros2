"""`python3 -m usv_froude_scaling` 入口（launch 找不到 console script 时的兜底）."""

from .cli import main

if __name__ == '__main__':
    raise SystemExit(main())
