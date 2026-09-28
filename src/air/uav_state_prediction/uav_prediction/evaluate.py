import argparse
import csv
import json
from pathlib import Path
import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .data import HORIZONS, baseline
from .model import load_predictor,predict


def metrics(pred,truth):
    e=pred-truth
    result=[]
    for i,h in enumerate(HORIZONS):
        row=dict(horizon_s=float(h),windows=len(e))
        for name,sl in [('position',slice(0,3)),('velocity',slice(3,6))]:
            d=e[:,i,sl]
            row[name+'_mae']=float(np.abs(d).mean())
            row[name+'_rmse']=float(np.sqrt((d*d).mean()))
            row[name+'_vector_rmse']=float(np.sqrt((d*d).sum(1).mean()))
        result.append(row)
    return result


def main():
    p=argparse.ArgumentParser(); p.add_argument('--data',type=Path,default=Path('data/processed'))
    p.add_argument('--models',type=Path,default=Path('models')); p.add_argument('--output',type=Path,default=Path('results')); a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True); torch.set_num_threads(3)
    test=np.load(a.data/'test.npz'); truth=test['truth']; records=[]; episode_records=[]; predictions={}
    for name,use_acc in [('constant_velocity',False),('constant_acceleration',True)]:
        predictions[name]=baseline(test['current'],use_acc)
    for ckpt_path in sorted(a.models.glob('*/model.pt')):
        model,ckpt=load_predictor(ckpt_path)
        predictions[ckpt_path.parent.name]=predict(model,ckpt,test['x'],test['current'])
    for name,pred in predictions.items():
        records += [dict(model=name,**row) for row in metrics(pred,truth)]
        for ep in np.unique(test['episode']):
            keep=test['episode']==ep
            episode_records += [dict(model=name,episode_id=str(ep),**row) for row in metrics(pred[keep],truth[keep])]
    for filename,rows in [('metrics.csv',records),('episode_metrics.csv',episode_records)]:
        with (a.output/filename).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    # Pick deployment seed only using validation loss; never pick it by test scores.
    configs=[(json.loads(path.read_text())['best_validation_loss'],path.parent.name) for path in a.models.glob('gru_*/config.json')]
    _,chosen=min(configs)
    summary={'deployment_model':chosen,'selection_rule':'minimum validation normalized MSE among GRU seeds',
        'metric_definition':'MAE/RMSE averaged over forecast windows and XYZ components; vector RMSE also reported',
        'units':{'position':'m','velocity':'m/s'},'test_episodes':np.unique(test['episode']).tolist(),
        'limitations':['Single Blocks scene; small pilot dataset','Overlapping windows are correlated; episode metrics supplied','No claim of real-flight generalization'],
        'seed_summary':[]}
    for variant in ['gru','no_acc_att','mlp']:
        for horizon in HORIZONS:
            rows=[r for r in records if r['model'].startswith(variant+'_') and r['horizon_s']==float(horizon)]
            summary['seed_summary'].append(dict(variant=variant,horizon_s=float(horizon),seeds=len(rows),
                position_rmse_mean=float(np.mean([r['position_rmse'] for r in rows])),
                position_rmse_std=float(np.std([r['position_rmse'] for r in rows])),
                velocity_rmse_mean=float(np.mean([r['velocity_rmse'] for r in rows]))))
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
    plt.style.use('seaborn-whitegrid')
    loss=np.genfromtxt(a.models/chosen/'loss.csv',delimiter=',',names=True)
    fig,ax=plt.subplots(figsize=(7,4)); ax.plot(loss['epoch'],loss['train_normalized_mse'],label='Train'); ax.plot(loss['epoch'],loss['validation_normalized_mse'],label='Validation')
    ax.set(xlabel='Epoch',ylabel='Normalized residual MSE',title='GRU training: '+chosen); ax.legend(); fig.tight_layout(); fig.savefig(a.output/'loss.png',dpi=160); plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for name in ['constant_velocity','constant_acceleration',chosen]:
        rows=[r for r in records if r['model']==name]
        for ax,key,unit in zip(axes,['position_rmse','velocity_rmse'],['m','m/s']):
            ax.plot(HORIZONS,[r[key] for r in rows],marker='o',label=name); ax.set(xlabel='Forecast horizon (s)',ylabel=key+' ('+unit+')')
    axes[0].legend(fontsize=8); fig.tight_layout(); fig.savefig(a.output/'horizon_errors.png',dpi=160); plt.close(fig)
    ep=np.unique(test['episode'])[1]; keep=test['episode']==ep; ts=test['t'][keep]+1
    fig,axes=plt.subplots(3,1,figsize=(9,7),sharex=True)
    for axis,ax in enumerate(axes):
        ax.plot(ts,truth[keep,-1,axis],label='Measured future')
        ax.plot(ts,predictions[chosen][keep,-1,axis],label='GRU forecast',linestyle='--')
        ax.plot(ts,predictions['constant_velocity'][keep,-1,axis],label='Constant velocity',alpha=0.6)
        ax.set_ylabel('NED '+ 'xyz'[axis]+' (m)')
    axes[0].set_title(ep+': 1-second forecasts aligned to target time'); axes[0].legend(); axes[-1].set_xlabel('Target simulation time within episode (s)'); fig.tight_layout(); fig.savefig(a.output/'prediction_comparison.png',dpi=160); plt.close(fig)
    rows=[r for r in summary['seed_summary'] if r['horizon_s']==1.0]
    fig,ax=plt.subplots(figsize=(7,4)); ax.bar([r['variant'] for r in rows],[r['position_rmse_mean'] for r in rows],yerr=[r['position_rmse_std'] for r in rows],capsize=5)
    ax.set(ylabel='Position RMSE at 1 s (m)',title='Ablation: mean and SD across training seeds'); fig.tight_layout(); fig.savefig(a.output/'ablation.png',dpi=160); plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__': main()
