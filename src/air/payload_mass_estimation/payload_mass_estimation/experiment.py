"""Collect, train and evaluate episode-separated payload estimation."""
import argparse,json,hashlib,os
from pathlib import Path
import numpy as np
import torch
from torch import nn
from .data import WINDOW,features
ROOT=Path(os.environ.get('PAYLOAD_PROJECT_ROOT',Path(__file__).resolve().parents[1])).resolve()

def collect_all():
    from .simulation import collect
    out=ROOT/'data';out.mkdir(exist_ok=True);manifest=[]
    for i in range(180):
        seed=40000+i;rng=np.random.default_rng(seed)
        mass=float(rng.uniform(0,.4));scale=1. if i<150 else float(rng.choice([.95,1.05]))
        truth=collect(seed,mass,thrust_scale=scale);observed=truth.copy()
        observed[:,2]+=rng.normal(0,.025,len(truth))
        split='train' if i<90 else 'validation' if i<120 else 'test' if i<150 else 'shift'
        file=out/f'episode_{seed}.npz';np.savez_compressed(file,observed=observed,truth=truth,mass_kg=mass)
        manifest.append({'file':file.name,'seed':seed,'split':split,'mass_kg':mass,'thrust_scale':scale,'sha256':hashlib.sha256(file.read_bytes()).hexdigest()})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));print('Collected 180 physical flights',flush=True)

def dataset(split):
    xs=[];ys=[];bs=[];ids=[]
    for item in json.loads((ROOT/'data/manifest.json').read_text()):
        if item['split']!=split:continue
        file=ROOT/'data'/item['file'];assert hashlib.sha256(file.read_bytes()).hexdigest()==item['sha256']
        with np.load(file) as a:
            for end in range(WINDOW,len(a['observed'])+1,10):
                x,b=features(a['observed'][end-WINDOW:end]);xs.append(x);ys.append(float(a['mass_kg']));bs.append(b);ids.append(item['seed'])
    return np.asarray(xs,np.float32),np.asarray(ys,np.float32),np.asarray(bs),np.asarray(ids)

def network(n):return nn.Sequential(nn.Linear(n,48),nn.Tanh(),nn.Linear(48,24),nn.Tanh(),nn.Linear(24,1))

def train():
    torch.set_num_threads(2)
    x,y,_,_=dataset('train');v,vy,_,_=dataset('validation')
    for name,seed,indices in [('mlp_42',42,list(range(38))),('mlp_43',43,list(range(38))),('mlp_44',44,list(range(38))),('no_thrust_42',42,list(range(19,38)))]:
        torch.manual_seed(seed);xx=x[:,indices];vv=v[:,indices];mean=xx.mean(0);std=xx.std(0).clip(.001)
        tx=torch.tensor((xx-mean)/std);tv=torch.tensor((vv-mean)/std);ty=torch.tensor(y[:,None]);tyv=torch.tensor(vy[:,None])
        net=network(len(indices));opt=torch.optim.Adam(net.parameters(),lr=.001);best=float('inf');history=[]
        dest=ROOT/'models'/name;dest.mkdir(parents=True,exist_ok=True)
        for epoch in range(160):
            net.train();perm=torch.randperm(len(tx))
            for batch in perm.split(128):
                opt.zero_grad();loss=((net(tx[batch])-ty[batch])**2).mean();loss.backward();opt.step()
            net.eval()
            with torch.no_grad():tl=float(((net(tx)-ty)**2).mean());vl=float(((net(tv)-tyv)**2).mean())
            history.append([epoch+1,tl,vl])
            if vl<best:best=vl;torch.save(net.state_dict(),dest/'model.pt')
        meta={'indices':indices,'mean':mean.tolist(),'std':std.tolist(),'seed':seed,'train_mean_kg':float(y.mean()),'best_val_mse':best}
        (dest/'meta.json').write_text(json.dumps(meta,indent=2));np.savetxt(dest/'loss.csv',history,delimiter=',',header='epoch,train_mse,val_mse',comments='');print(name,best,flush=True)

def predictor(name='mlp_42'):
    dest=ROOT/'models'/name;meta=json.loads((dest/'meta.json').read_text());net=network(len(meta['indices']));net.load_state_dict(torch.load(dest/'model.pt',map_location='cpu',weights_only=True));net.eval()
    def run(x):
        x=np.asarray(x)[...,meta['indices']];x=(x-np.asarray(meta['mean']))/np.asarray(meta['std'])
        with torch.no_grad():return net(torch.tensor(x,dtype=torch.float32)).numpy().squeeze(-1)
    return run

def evaluate():
    out=ROOT/'results';out.mkdir(exist_ok=True);results={}
    for split in ['test','shift']:
        x,y,b,ids=dataset(split);models={n:predictor(n)(x) for n in ['mlp_42','mlp_43','mlp_44','no_thrust_42']};models['physics_impulse']=b
        models['train_mean']=np.full_like(y,json.loads((ROOT/'models/mlp_42/meta.json').read_text())['train_mean_kg']);results[split]={}
        for name,pred in models.items():
            errors=pred-y;flight_errors=np.asarray([errors[ids==i].mean() for i in np.unique(ids)])
            results[split][name]={'window_mae_kg':float(np.abs(errors).mean()),'window_rmse_kg':float(np.sqrt((errors**2).mean())),'flight_mean_mae_kg':float(np.abs(flight_errors).mean()),'flights':len(np.unique(ids)),'windows':len(y)}
            np.savetxt(out/f'{split}_{name}.csv',np.c_[ids,y,pred],delimiter=',',header='flight,true_kg,pred_kg',comments='')
    (out/'metrics.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    loss_path=ROOT/'models/mlp_42/loss.csv'
    if loss_path.exists():
        fig,ax=plt.subplots();loss=np.loadtxt(ROOT/'models/mlp_42/loss.csv',delimiter=',',skiprows=1);ax.plot(loss[:,0],loss[:,1],label='train');ax.plot(loss[:,0],loss[:,2],label='validation');ax.set(xlabel='Epoch',ylabel='MSE (kg squared)');ax.legend();fig.tight_layout();fig.savefig(out/'loss.png',dpi=150);plt.close(fig)
    else:
        print('Training log not present; skipping loss plot. Run train to generate a new log.')
    fig,ax=plt.subplots()
    for name in ['mlp_42','physics_impulse']:
        a=np.loadtxt(out/f'test_{name}.csv',delimiter=',',skiprows=1);means=np.asarray([a[a[:,0]==i,1:].mean(0) for i in np.unique(a[:,0])]);ax.scatter(means[:,0],means[:,1],label=name,alpha=.7)
    ax.plot([0,.4],[0,.4],'k--');ax.set(xlabel='True payload (kg)',ylabel='Flight mean estimate (kg)');ax.legend();fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['collect','train','evaluate']);a=p.parse_args();{'collect':collect_all,'train':train,'evaluate':evaluate}[a.stage]()
