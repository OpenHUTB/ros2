"""Validate the selected ROS Python environment and use colcon's public entry point."""
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys

MODULES = ['numpy', 'torch', 'matplotlib', 'rclpy', 'std_msgs', 'lark', 'launch_ros',
           'colcon_core', 'colcon_ros', 'colcon_python_setup_py',
           'colcon_package_selection', 'colcon_recursive_crawl']


def doctor(write=False):
    missing = []
    for module in MODULES:
        try:
            importlib.import_module(module)
        except ImportError as error:
            missing.append(module + ': ' + str(error))
    if missing:
        raise SystemExit('Environment incomplete for ' + sys.executable + '\n' + '\n'.join(missing)
                         + '\nSource ROS2 and activate a /usr/bin/python3 venv created with --system-site-packages.'
                         + '\nThen run: python -m pip install -r requirements.txt')
    versions = {}
    for name in ['numpy', 'torch', 'matplotlib', 'lark', 'pip', 'setuptools', 'colcon-core', 'colcon-common-extensions']:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = 'not installed as a distribution'
    report = {'python': sys.version, 'executable': sys.executable, 'ros_distro': os.environ.get('ROS_DISTRO'),
              'packages': versions, 'required_imports': MODULES, 'passed': True}
    print(json.dumps(report, indent=2))
    if write:
        folder = Path(__file__).resolve().parent / 'artifacts'
        folder.mkdir(exist_ok=True)
        (folder / 'environment.json').write_text(json.dumps(report, indent=2))
    return report


def build(package, root):
    doctor()
    # colcon is a console script. Standard installations expose colcon_core,
    # but do not necessarily provide an importable `colcon` module.
    command = [sys.executable, '-c', 'from colcon_core.command import main; raise SystemExit(main())',
               'build', '--base-paths', str(root), '--packages-select', package]
    subprocess.run(command, cwd=root, check=True)


if __name__ == '__main__':
    doctor(write='--write' in sys.argv)
