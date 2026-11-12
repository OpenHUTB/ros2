#!/bin/bash
# One-click entry for the DAVE underwater simulation homework module.
# Usage:
#   bash main.sh                 # default: demo scene, GUI
#   bash main.sh valve           # BOP panel demo scene, GUI
#   bash main.sh demo headless   # no GUI (gzserver only)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/uuv_ws}

if [ ! -d "$WS/src/dave" ] || [ ! -f "$WS/devel/setup.bash" ]; then
  echo "DAVE workspace not found at $WS."
  echo "run '$HERE/install_dave.sh' first, or set WS=/path/to/workspace"
  exit 1
fi

MODE=${1:-demo}
shift || true
exec bash "$HERE/run_demo.sh" "$MODE" "$@"
