#!/usr/bin/env bash
# MuJoCo 无人车自动泊车一键入口（课程要求的 main.* 入口）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# WSL 下 MuJoCo 的 GLFW/EGL 离屏后端可能段错误：
# 无窗口模式用 osmesa，GUI 模式用 glfw + 软件光栅化。
IS_WSL=0
if grep -qi microsoft /proc/version 2>/dev/null || [ -n "$WSL_DISTRO_NAME" ]; then
    IS_WSL=1
fi
GUI_MODE=0
for arg in "$@"; do
    case "$arg" in
        --gui) GUI_MODE=1 ;;
    esac
done
if [ "$IS_WSL" = "1" ] && [ "$GUI_MODE" = "0" ] && [ -z "$MUJOCO_GL" ]; then
    export MUJOCO_GL=osmesa
fi
if [ "$IS_WSL" = "1" ] && [ "$GUI_MODE" = "1" ] && [ -z "$LIBGL_ALWAYS_SOFTWARE" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    export MUJOCO_GL=glfw
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
for WS in "$HOME/auto_parking_ws" "$HOME/ros2_ws"; do
    if [ -f "$WS/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "$WS/install/setup.bash"
        break
    fi
done

cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:${PYTHONPATH:-}"
exec python3 main.py "$@"
