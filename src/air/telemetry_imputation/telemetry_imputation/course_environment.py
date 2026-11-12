"""Environment checks and colcon build for the telemetry-imputation module."""
import importlib
import importlib.metadata
import json
import subprocess
import sys
from pathlib import Path

MODULES = ['numpy', 'torch', 'matplotlib', 'rclpy', 'std_msgs', 'lark', 'launch_ros',
           'colcon_core', 'colcon_ros', 'colcon_python_setup_py',
           'colcon_package_selection', 'colcon_recursive_crawl']


def doctor(write=False):
    missing = []
    for module in MODULES:
        try:
            importlib.import_module(module)
        except ImportError as error:
            missing.append('%s: %s' % (module, error))
    packages = {}
    for name in ('numpy', 'torch', 'matplotlib', 'colcon-core', 'colcon-common-extensions'):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    report = {
        'python': sys.version.split()[0],
        'executable': sys.executable,
        'packages': packages,
        'required_imports': MODULES,
        'missing': missing,
        'passed': not missing,
    }
    if write:
        out = Path('artifacts')
        out.mkdir(exist_ok=True)
        (out / 'environment.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if missing:
        print('Environment incomplete: install requirements.txt inside a ROS-enabled venv.')
        raise SystemExit(1)


def build(package, root):
    """Drive colcon through its public entry point (it reads sys.argv)."""
    from colcon_core.command import main as colcon_main
    doctor(write=False)
    saved = sys.argv
    sys.argv = ['colcon', 'build', '--base-paths', str(Path(root)),
                '--packages-select', package, '--symlink-install']
    try:
        code = colcon_main()
    finally:
        sys.argv = saved
    if code:
        raise SystemExit(code)
    print('Build finished: source install/setup.bash')


if __name__ == '__main__':
    doctor(write=True)
