import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .data import PHASES,observable,WINDOW
from .learning import load,predict

def main():
    out=Path('results');out.mkdir(exist_ok=True)
    metrics=json.loads((out/'metrics.json').read_text())
    split='test';data=metrics[split];names=list(data)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    axes[0].bar(range(len(names)),[data[n]['macro_f1'] for n in names]);axes[0].set_ylabel('Phase macro F1');axes[0].set_ylim(0,1)
    axes[1].bar(range(len(names)),[data[n]['impact_mae'] for n in names]);axes[1].set_ylabel('Peak load factor MAE (body weights)')
    for ax in axes:ax.set_xticks(range(len(names)));ax.set_xticklabels(names,rotation=35,ha='right');ax.grid(axis='y',alpha=.2)
    fig.suptitle('Episode-held-out test: networks, ablation and baselines');fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
    loss=np.genfromtxt('models/trained/mlp_42/loss.csv',delimiter=',',names=True)
    fig,ax=plt.subplots(figsize=(7,4));ax.plot(loss['epoch'],loss['train_loss'],label='Train: weighted CE + impact MSE');ax.plot(loss['epoch'],loss['val_loss'],label='Validation: CE + impact MSE');ax.set(xlabel='Epoch',ylabel='Loss');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'loss.png',dpi=150);plt.close(fig)
    matrix=np.array(data['mlp_42']['confusion']);fig,ax=plt.subplots(figsize=(6,5));ax.imshow(matrix,cmap='Blues')
    for i in range(4):
        for j in range(4):ax.text(j,i,str(matrix[i,j]),ha='center',va='center',color='black')
    ax.set_xticks(range(4));ax.set_xticklabels(PHASES,rotation=25);ax.set_yticks(range(4));ax.set_yticklabels(PHASES);ax.set(xlabel='Predicted',ylabel='Contact-derived label');fig.tight_layout();fig.savefig(out/'confusion.png',dpi=150);plt.close(fig)
    with (out/'impact_predictions.csv').open() as f:rows=[r for r in csv.DictReader(f) if r['split']=='test']
    fig,ax=plt.subplots(figsize=(6,5))
    for name in ('mlp_42','speed_quadratic_rules'):
        selected=[r for r in rows if r['model']==name];x=[float(r['true_peak_load_factor']) for r in selected];y=[float(r['predicted_peak_load_factor']) for r in selected];ax.scatter(x,y,label=name,alpha=.7)
    ax.plot([0,30],[0,30],'k--');ax.set(xlabel='Actual peak load factor',ylabel='Pre-contact predicted peak load factor');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'impact_comparison.png',dpi=150);plt.close(fig)
    manifest=json.loads(Path('data/raw/manifest.json').read_text());record=next(r for r in manifest if r['split']=='test')
    raw=np.load(Path('data/raw')/record['file'])['raw'];features=observable(raw);net,meta=load('models/trained/mlp_42')
    x=np.array([features[i-WINDOW+1:i+1] for i in range(WINDOW-1,len(raw))]);prob,_=predict(net,meta,x)
    fig,axes=plt.subplots(2,1,figsize=(9,5),sharex=True);axes[0].plot(raw[:,0],features[:,0]);axes[0].axvline(record['contact_time_s'],color='r',ls='--',label='Contact truth (evaluation only)');axes[0].set_ylabel('Foot clearance (m)');axes[0].legend()
    axes[1].step(raw[:,0],raw[:,15],where='post',label='Actual');axes[1].step(raw[WINDOW-1:,0],prob.argmax(1),where='post',label='Neural',alpha=.8);axes[1].set_yticks(range(4));axes[1].set_yticklabels(PHASES);axes[1].set_xlabel('Simulation seconds');axes[1].legend();fig.tight_layout();fig.savefig(out/'phase_comparison.png',dpi=150);plt.close(fig)

if __name__=='__main__':main()
