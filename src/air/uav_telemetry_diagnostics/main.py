"""Portable project entry. Run from source; packaged ROS entrypoints are also provided."""
import argparse
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def prepare():
    from telemetry_data import prepare as make_data
    target = ROOT / 'data'
    with zipfile.ZipFile(ROOT / 'assets/flight_corpus.zip') as archive:
        for info in archive.infolist():
            destination = (target / info.filename).resolve()
            if target.resolve() not in destination.parents:
                raise ValueError('Unsafe archive path')
        archive.extractall(target)
    records = make_data(target / 'original', target / 'prepared')
    print('Prepared', len(records), 'scenarios from checksum-verified raw flights')


def main():
    parser = argparse.ArgumentParser(description='UAV telemetry diagnostics course project')
    parser.add_argument('command', choices=['prepare', 'train', 'evaluate', 'report', 'test', 'build', 'demo', 'live', 'dashboard'])
    args, extra = parser.parse_known_args()
    if args.command == 'prepare':
        prepare(); return
    scripts = {'train': 'train_consistency.py', 'evaluate': 'evaluate_consistency.py', 'report': 'report_final.py',
               'dashboard': 'dashboard.py', 'live': 'live_source.py'}
    if args.command in scripts:
        cmd = [sys.executable, scripts[args.command]] + extra
    elif args.command == 'test':
        cmd = [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v']
    elif args.command == 'build':
        cmd = [sys.executable, '-m', 'colcon', 'build', '--base-paths', '.', '--packages-select', 'uav_telemetry_diagnostics']
    else:
        cmd = ['bash', 'main.sh', 'demo'] + extra
    subprocess.run(cmd, cwd=ROOT, check=True)


if __name__ == '__main__': main()
