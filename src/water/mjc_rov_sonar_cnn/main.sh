#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa
cd "$SCRIPT_DIR"
exec python3 -m mjc_rov_sonar_cnn.main "$@"
