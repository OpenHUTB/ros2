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
# 选择要 source 的工作空间，优先级从高到低：
#   1) 本模块所在的工作空间（从脚本路径向上查找 install/setup.bash）
#   2) ~/ros2_ws
#   3) ~/auto_parking_ws
# 这样同时存在多个工作空间时，不会 source 到别处的旧安装。
find_ws_from_script() {
    local dir="$SCRIPT_DIR"
    local i=0
    while [ "$i" -lt 4 ]; do
        if [ -f "$dir/install/setup.bash" ]; then
            printf '%s\n' "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
        i=$((i + 1))
    done
    return 1
}

WS_SOURCED=""
if WS_FROM_SCRIPT="$(find_ws_from_script)"; then
    WS_SOURCED="$WS_FROM_SCRIPT"
else
    for WS in "$HOME/ros2_ws" "$HOME/auto_parking_ws"; do
        if [ -f "$WS/install/setup.bash" ]; then
            WS_SOURCED="$WS"
            break
        fi
    done
fi
if [ -n "$WS_SOURCED" ]; then
    # shellcheck disable=SC1090
    source "$WS_SOURCED/install/setup.bash"
fi

cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:${PYTHONPATH:-}"
exec python3 main.py "$@"
