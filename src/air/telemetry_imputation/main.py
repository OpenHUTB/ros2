"""Portable entry point for the telemetry-imputation module."""
import argparse
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def prepare():
    """Unpack the shipped corpus and verify every checksum."""
    target = ROOT / 'flights'
    target.mkdir(exist_ok=True)
    with zipfile.ZipFile(ROOT / 'assets/flights.zip') as archive:
        for info in archive.infolist():
            destination = (target / info.filename).resolve()
            if target.resolve() not in destination.parents:
                raise ValueError('Unsafe archive path')
        archive.extractall(target)
    import hashlib
    import json
    manifest = json.loads((target / 'manifest.json').read_text())
    for item in manifest:
        digest = hashlib.sha256((target / item['file']).read_bytes()).hexdigest()
        if digest != item['sha256']:
            raise ValueError('Checksum mismatch: ' + item['file'])
    print('Prepared %d flights from checksum-verified corpus' % len(manifest))


def main():
    parser = argparse.ArgumentParser(description='UAV telemetry imputation')
    parser.add_argument('command', choices=['doctor', 'prepare', 'collect', 'train', 'evaluate',
                                            'report', 'test', 'build', 'demo'])
    args, extra = parser.parse_known_args()
    if args.command == 'doctor':
        from telemetry_imputation.course_environment import doctor
        doctor(write=True)
        return
    if args.command == 'prepare':
        prepare()
        return
    if args.command == 'build':
        from telemetry_imputation.course_environment import build
        build('telemetry_imputation', ROOT)
        return
    scripts = {
        'collect': 'telemetry_imputation.gen',
        'train': 'telemetry_imputation.train',
        'evaluate': 'telemetry_imputation.evaluate',
        'report': 'telemetry_imputation.report',
    }
    if args.command in scripts:
        cmd = [sys.executable, '-m', scripts[args.command]] + extra
    elif args.command == 'test':
        cmd = [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v']
    else:
        cmd = ['bash', 'demo.sh'] + extra
    subprocess.run(cmd, cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
