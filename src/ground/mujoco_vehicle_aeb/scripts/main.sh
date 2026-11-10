#!/bin/bash
# 切换到脚本所在目录 scripts
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
# 向上回到catkin包根目录
PACKAGE_ROOT=$(dirname "${SCRIPT_DIR}")

# 进入包根，保证models相对路径正确
cd "${PACKAGE_ROOT}"

# 虚拟机/无显卡：软件渲染
export MJ_CORE_GLFW=software

# 可选：尝试source catkin环境，兼容ROS
if [ -f devel/setup.bash ];then
    source devel/setup.bash
fi

# 在包根执行main.py
python3 "${SCRIPT_DIR}/main.py"
