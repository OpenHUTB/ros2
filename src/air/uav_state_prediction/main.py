#!/usr/bin/env python3
"""Single module entry point; execute from an activated ROS2 Python environment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser()
    p.add_argument('stage',choices=['collect','prepare','train','evaluate','test','build','demo','live'])
    p.add_argument('--rviz',action='store_true'); p.add_argument('--host',default='192.168.239.1')
    a,extra=p.parse_known_args(); os.chdir(ROOT)
    if a.stage in ['collect','prepare','train','evaluate']:
        module={'prepare':'data'}.get(a.stage,a.stage)
        cmd=[sys.executable,'-m','uav_prediction.'+module]+extra
        if a.stage=='collect': cmd+=['--host',a.host]
    elif a.stage=='test': cmd=[sys.executable,'-m','unittest','discover','-s','tests','-v']
    elif a.stage=='build': cmd=[sys.executable,'-m','colcon','build','--base-paths',str(ROOT),'--packages-select','uav_state_prediction']
    else:
        summary=json.loads((ROOT/'results/summary.json').read_text())
        model=ROOT/'models'/summary['deployment_model']/'model.pt'
        cmd=['ros2','launch','uav_state_prediction','main.launch.py','model:='+str(model),
             'host:='+a.host,'rviz:='+str(a.rviz).lower(),'plot:='+str(not a.rviz).lower(),
             'error_log:='+str(ROOT/'results/online_errors.jsonl')]
        if a.stage=='demo': cmd+=['csv:='+str(ROOT/'data/sample/episode_026.csv')]
        cmd+=extra
    subprocess.run(cmd,check=True)


if __name__=='__main__': main()
