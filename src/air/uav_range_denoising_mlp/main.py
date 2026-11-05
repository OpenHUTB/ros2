#!/usr/bin/env python3
import argparse, csv, json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))

def main():
    parser=argparse.ArgumentParser(description='Synthetic range denoising; --ros publishes estimates only')
    parser.add_argument('--ros',action='store_true');parser.add_argument('--output',default='outputs')
    args,_=parser.parse_known_args()
    if args.ros:
        from ros_node import run
        run();return
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from core import dataset,train,predict
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=False)
    x,y,_,_=dataset(101);model=train(x,y)
    tx,ty,noisy,mean=dataset(909,sequences=5)
    estimated=predict(tx,model).ravel();truth=ty.ravel()*4
    rmse=lambda values:float(np.sqrt(np.mean((values-truth)**2)))
    metrics={'raw_rmse_m':rmse(noisy),'mean_rmse_m':rmse(mean),'mlp_rmse_m':rmse(estimated),
             'train_seed':101,'test_seed':909,'test_samples':len(truth),
             'scope':'synthetic independent sequences; no ROS or real flight validation'}
    (folder/'model.json').write_text(json.dumps([v.tolist() for v in model]))
    (folder/'metrics.json').write_text(json.dumps(metrics,indent=2))
    with (folder/'samples.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['truth_m','raw_m','mean_m','mlp_m']);w.writerows(zip(truth,noisy,mean,estimated))
    fig,ax=plt.subplots(figsize=(8,4));n=154
    ax.plot(truth[:n],label='Synthetic truth');ax.plot(noisy[:n],alpha=.4,label='Noisy range')
    ax.plot(mean[:n],label='Causal mean baseline');ax.plot(estimated[:n],label='MLP')
    ax.set(title='Offline synthetic test sequence (not flight data)',xlabel='Sample',ylabel='Range (m)')
    ax.grid(True,alpha=.3);ax.legend();fig.tight_layout();fig.savefig(folder/'offline_demo.svg');plt.close(fig)
    print(json.dumps(metrics,indent=2))

if __name__=='__main__':main()
