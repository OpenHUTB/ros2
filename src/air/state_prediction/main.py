#!/usr/bin/env python3
"""Single module entry point; execute from an activated ROS2 Python environment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parent


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'doctor':
        from course_environment import doctor
        doctor(write=True)
        return
    p=argparse.ArgumentParser()
    p.add_argument('stage',choices=['doctor', 'collect','prepare','train','evaluate','test','build','demo','live'])
    p.add_argument('--rviz',action='store_true'); p.add_argument('--host',default='192.168.239.1')
    a,extra=p.parse_known_args(); os.chdir(ROOT)
    if a.stage == 'prepare' and not extra and not (ROOT/'data/raw').exists():
        target=ROOT/'data'
        with zipfile.ZipFile(ROOT/'assets/flight_corpus.zip') as archive:
            for item in archive.infolist():
                if target.resolve() not in (target/item.filename).resolve().parents:
                    raise ValueError('Unsafe corpus path')
            archive.extractall(target)
    if a.stage in ['collect','prepare','train','evaluate']:
        module={'prepare':'data'}.get(a.stage,a.stage)
        cmd=[sys.executable,'-m','prediction.'+module]+extra
        if a.stage=='collect': cmd+=['--host',a.host]
    elif a.stage=='test': cmd=[sys.executable,'-m','unittest','discover','-s','tests','-v']
    elif a.stage=='build':
        from course_environment import build
        build('state_prediction',ROOT)
        return
    else:
        summary=json.loads((ROOT/'results/summary.json').read_text())
        model=ROOT/'models'/summary['deployment_model']/'model.pt'
        cmd=['ros2','launch','state_prediction','main.launch.py','model:='+str(model),
             'host:='+a.host,'rviz:='+str(a.rviz).lower(),'plot:='+str(not a.rviz).lower(),
             'error_log:='+str(ROOT/'results/online_errors.jsonl')]
        if a.stage=='demo': cmd+=['csv:='+str(ROOT/'data/sample/episode_026.csv')]
        cmd+=extra
    try:
        subprocess.run(cmd,check=True)
    except KeyboardInterrupt:
        return


if __name__=='__main__': main()
