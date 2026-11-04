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
    from core import transform, inverse
    rows=[]
    for i in range(101):
     angle=i*2*math.pi/100; p=(math.cos(angle),math.sin(angle))
     q=transform(p,(2.,-1.),.7); back=inverse(q,(2.,-1.),.7)
     rows.append((*p,*q,math.hypot(p[0]-back[0],p[1]-back[1])))
    header=['local_x','local_y','world_x','world_y','roundtrip_error']
    ax.plot([r[0] for r in rows],[r[1] for r in rows],label='Local coordinates')
    ax.plot([r[2] for r in rows],[r[3] for r in rows],label='World coordinates')
    ax.axis('equal'); ax.set(xlabel='x',ylabel='y')
    summary={'max_roundtrip_error':max(r[4] for r in rows),'samples':len(rows)}
    assert summary['max_roundtrip_error']<1e-12
    ax.set_title('Offline synthetic demonstration (not ROS runtime)')
    ax.grid(True,alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(folder/'offline_demo.svg'); plt.close(fig)
    with (folder/'samples.csv').open('w',newline='') as f:
        writer=csv.writer(f); writer.writerow(header); writer.writerows(rows)
    summary['validation_scope']='offline mathematical demo; ROS runtime not tested'
    (folder/'metrics.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
