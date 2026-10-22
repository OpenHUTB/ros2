"""定位源码或安装目录中的资源."""
from pathlib import Path


def resource(relative):
    """优先查找源码目录，再查找 ament 安装目录."""
    source = Path(__file__).resolve().parents[1] / relative
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('uav_altitude_command_guard')) / relative
