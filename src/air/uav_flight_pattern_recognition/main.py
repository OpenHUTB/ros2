"""Run from any working directory; all artifacts stay inside this project."""
import argparse
import os
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def prepare():
    from pattern_data import prepare as make_data
    target = ROOT / 'data'
    with zipfile.ZipFile(ROOT / 'assets/flight_corpus.zip') as archive:
        for info in archive.infolist():
            if target.resolve() not in (target / info.filename).resolve().parents:
                raise ValueError('Unsafe archive path')
        archive.extractall(target)
    print(make_data(target))
    if (target / 'final_raw').exists():
        from prepare_final import main as make_final
        make_final()


def main():
    parser = argparse.ArgumentParser(description='UAV flight pattern recognition')
    parser.add_argument('command', choices=['prepare', 'train', 'reproduce', 'evaluate', 'report', 'test', 'build', 'demo', 'dashboard', 'live'])
    args, extra = parser.parse_known_args()
    os.chdir(ROOT)
    if args.command == 'prepare':
        prepare()
        return
    if args.command == 'test':
        command = [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v']
    elif args.command == 'build':
        command = [sys.executable, '-m', 'colcon', 'build', '--base-paths', '.', '--packages-select', 'uav_flight_pattern_recognition']
    elif args.command == 'demo':
        command = ['bash', 'main.sh', 'demo']
    else:
        script = {'dashboard': 'pattern_dashboard', 'live': 'pattern_live'}.get(args.command, args.command)
        command = [sys.executable, script + '.py']
    subprocess.run(command + extra, check=True)


if __name__ == '__main__':
    main()
