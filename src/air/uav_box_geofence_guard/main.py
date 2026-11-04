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
    from core import guard
    x=0.; rows=[]
    for i in range(240):
        t=i*.05
        values,state=guard((x,0.,2.),(1.5,0.,0.,0.),0.,0.)
        rows.append((t,x,values[0],state)); x+=values[0]*.05
    header=['time_s','x_m','vx_safe','state']
    ax.plot([r[0] for r in rows],[r[1] for r in rows],label='Synthetic x')
    ax.axhline(4.85,color='red',linestyle='--',label='Boundary')
    ax.set(xlabel='Time (s)',ylabel='x (m)')
    summary={'final_x':x,'boundary':4.85,'limited_steps':sum(r[3]=='LIMITED' for r in rows)}
    assert x<=4.85+1e-10
    ax.set_title('Offline synthetic demonstration (not ROS runtime)')
    ax.grid(True,alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(folder/'offline_demo.svg'); plt.close(fig)
    with (folder/'samples.csv').open('w',newline='') as f:
        writer=csv.writer(f); writer.writerow(header); writer.writerows(rows)
    summary['validation_scope']='offline mathematical demo; ROS runtime not tested'
    (folder/'metrics.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
