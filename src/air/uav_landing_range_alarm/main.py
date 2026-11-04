#!/usr/bin/env python3
"""Offline mathematical demonstration, or --ros for a ROS Noetic node."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--output',default='outputs')
    args,_=parser.parse_known_args()  # roslaunch appends remapping arguments
    if args.ros:
        from ros_node import run
        run()
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(args.output)
    folder.mkdir(parents=True,exist_ok=False)
    fig,ax=plt.subplots(figsize=(7,4))
    from core import Alarm
    alarm=Alarm(); rows=[]; codes={'UNKNOWN':0,'CLEAR':1,'ALARM':2,'STALE':3,'INVALID':4}
    for i in range(250):
     t=i*.05
     value=.8 if 3.<t<6. else (1.15+.08*math.sin(12*t) if t<8 else 2.)
     if not 8.<t<9.5: alarm.update(value,t)
     state=alarm.check(t)
     rows.append((t,value,state))
    header=['time_s','synthetic_range_m','state']
    ax.plot([r[0] for r in rows],[r[1] for r in rows],label='Synthetic range (m)')
    ax.step([r[0] for r in rows],[codes[r[2]] for r in rows],where='post',label='State code (0/1/2/3/4)')
    ax.set(xlabel='Time (s)',ylabel='Range / state code')
    summary={'alarm_samples':sum(r[2]=='ALARM' for r in rows),'stale_samples':sum(r[2]=='STALE' for r in rows),'final_state':rows[-1][2]}
    assert summary['alarm_samples']>0 and summary['stale_samples']>0 and rows[-1][2]=='CLEAR'
    ax.set_title('Offline synthetic demonstration (not ROS runtime)')
    ax.grid(True,alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(folder/'offline_demo.svg'); plt.close(fig)
    with (folder/'samples.csv').open('w',newline='') as f:
        writer=csv.writer(f); writer.writerow(header); writer.writerows(rows)
    summary['validation_scope']='offline mathematical demo; ROS runtime not tested'
    (folder/'metrics.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
