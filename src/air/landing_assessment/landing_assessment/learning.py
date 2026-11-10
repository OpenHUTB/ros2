"""Train-only normalization, fixed validation selection, episode-held-out evaluation."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from .data import prepare,WINDOW,PHASES

# 载荷预测区间的目标覆盖率：1-alpha = 90%
ALPHA=0.1

class Network(nn.Module):
    def __init__(self,kind='mlp',channels=12):
        super().__init__();self.kind=kind
        self.encoder=(nn.GRU(channels,24,batch_first=True) if kind=='gru' else
                      nn.Sequential(nn.Flatten(),nn.Linear(WINDOW*channels,48),nn.Tanh(),nn.Linear(48,24),nn.Tanh()))
        self.phase=nn.Linear(24,4);self.impact=nn.Linear(24,1)
    def forward(self,x):
        z=self.encoder(x)
        if self.kind=='gru':z=z[0][:,-1]
        return self.phase(z),self.impact(z).squeeze(-1)

def tensor(x):return torch.as_tensor(x,dtype=torch.float32)

def normalized(x,meta):
    z=x[:,:,meta['channels']]
    return np.clip((z-np.asarray(meta['mean']))/np.asarray(meta['std']),-10,10).astype(np.float32)

def load(folder):
    folder=Path(folder);meta=json.loads((folder/'meta.json').read_text(encoding='utf-8'))
    net=Network(meta['kind'],len(meta['channels']))
    net.load_state_dict(torch.load(folder/'model.pt',map_location='cpu',weights_only=True));net.eval()
    return net,meta

def predict(net,meta,x):
    with torch.no_grad():
        logits,value=net(tensor(normalized(x,meta)))
    quality=np.maximum(0,np.expm1(value.numpy()*meta['qstd']+meta['qmean']))
    return logits.softmax(-1).numpy(),quality


def conformal_quantile(residuals,alpha=ALPHA):
    """split conformal：在标定集残差上取第 ceil((n+1)(1-alpha)) 小的绝对残差。

    返回 (q, 次序统计量序号, 标定样本数)。q 的含义是：新的预测值有约 1-alpha 的概率
    落在 ±q 之内，前提是标定样本与新样本可交换（同一仿真分布、同一切分流程）。
    """
    values=np.sort(np.abs(np.asarray(residuals,dtype=float)))
    n=len(values)
    if n==0:raise ValueError('Calibration set is empty')
    order=min(n,int(np.ceil((n+1)*(1-alpha))))
    return float(values[order-1]),order,n


def interval(meta,point):
    """把点预测扩展成标定后的闭区间；未标定的旧模型退化为退化区间。"""
    point=float(point)
    conformal=meta.get('conformal') or {}
    q=conformal.get('q')
    if q is None:return point,point
    return max(0.,point-float(q)),point+float(q)

def train(folder='data/raw',output='models/trained',epochs=100):
    torch.set_num_threads(1);sets=prepare(folder);tr,va=sets['train'],sets['val']
    for kind,channels,seed,label in [('mlp',list(range(12)),s,f'mlp_{s}') for s in (42,43,44)]+[
            ('gru',list(range(12)),42,'gru_42'),('mlp',[0,1,2,3],42,'kinematics_only_42')]:
        torch.manual_seed(seed);out=Path(output)/label;out.mkdir(parents=True,exist_ok=True)
        logq=np.log1p(tr['qy']);meta={'kind':kind,'channels':channels,'seed':seed,
              'mean':tr['x'][:,:,channels].mean((0,1)).tolist(),
              'std':np.maximum(tr['x'][:,:,channels].std((0,1)),.01).tolist(),
              'qmean':float(logq.mean()),'qstd':float(max(logq.std(),.01)),
              'window':WINDOW,'selection':'minimum validation CE + normalized impact MSE',
              'training_episode_ids':tr['qepisode'].tolist(),'validation_episode_ids':va['qepisode'].tolist()}
        net=Network(kind,len(channels));optimizer=torch.optim.Adam(net.parameters(),lr=.003)
        px=tensor(normalized(tr['x'],meta));py=torch.tensor(tr['y'],dtype=torch.long)
        qx=tensor(normalized(tr['qx'],meta));qy=tensor((logq-meta['qmean'])/meta['qstd'])
        vx=tensor(normalized(va['x'],meta));vy=torch.tensor(va['y'],dtype=torch.long)
        vqx=tensor(normalized(va['qx'],meta));vqy=tensor((np.log1p(va['qy'])-meta['qmean'])/meta['qstd'])
        weights=1/np.maximum(np.bincount(tr['y'],minlength=4),1);weights=weights/weights.mean()
        criterion=nn.CrossEntropyLoss(weight=tensor(weights));best=float('inf');history=[]
        for epoch in range(1,epochs+1):
            net.train();optimizer.zero_grad()
            logits,_=net(px);_,estimate=net(qx)
            loss=criterion(logits,py)+nn.functional.mse_loss(estimate,qy)
            loss.backward();nn.utils.clip_grad_norm_(net.parameters(),5);optimizer.step()
            net.eval()
            with torch.no_grad():
                vl,_=net(vx);_,vq=net(vqx)
                score=nn.functional.cross_entropy(vl,vy)+nn.functional.mse_loss(vq,vqy)
            history.append([epoch,float(loss),float(score)])
            if float(score)<best:
                best=float(score);torch.save(net.state_dict(),out/'model.pt');meta['best_epoch']=epoch
        # 用验证集标定载荷预测区间（best checkpoint，而非最后一轮权重）
        net.load_state_dict(torch.load(out/'model.pt',map_location='cpu',weights_only=True));net.eval()
        with torch.no_grad():
            _,vq_best=net(vqx)
        predicted=np.maximum(0,np.expm1(vq_best.numpy()*meta['qstd']+meta['qmean']))
        q,order,n=conformal_quantile(predicted-va['qy'])
        meta['conformal']={'alpha':ALPHA,'coverage_target':1-ALPHA,'q':q,
                           'order_statistic':order,'calibration_episodes':n,
                           'policy':'split conformal on validation episodes, absolute residual in body weights'}
        # 风险分级阈值取训练集峰值载荷的 P75/P90，避免手填物理量
        meta['risk_thresholds']={'elevated':float(np.percentile(tr['qy'],75)),
                                 'high':float(np.percentile(tr['qy'],90))}
        (out/'meta.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
        np.savetxt(out/'loss.csv',history,delimiter=',',header='epoch,train_loss,val_loss',comments='')
        print(label,'validation loss',round(best,4),'epoch',meta['best_epoch'],flush=True)
    # Fit the physical-feature regression once, using only the training episodes.
    def design(x):
        vz=x[:,-1,3];return np.c_[np.ones(len(vz)),abs(vz),vz*vz]
    matrix=design(tr['qx']);coef=np.linalg.solve(matrix.T@matrix+.001*np.eye(3),matrix.T@tr['qy'])
    (Path(output)/'baselines.json').write_text(json.dumps({'mean':float(tr['qy'].mean()),'speed_quadratic':coef.tolist()}),encoding='utf-8')

def metrics(y,p):
    confusion=np.zeros((4,4),int)
    for a,b in zip(y,p):confusion[int(a),int(b)]+=1
    f1=2*np.diag(confusion)/np.maximum(confusion.sum(0)+confusion.sum(1),1)
    return {'accuracy':float(np.mean(y==p)),'macro_f1':float(f1.mean()),'confusion':confusion.tolist()}

def rules(x):
    last=x[:,-1];h=last[:,0];vz=last[:,3]
    return np.where(h>.04,np.where(vz<-.12,1,0),np.where(np.linalg.norm(last[:,1:4],axis=1)<.10,3,2))

def evaluate(folder='data/raw',models='models/trained',output='results'):
    torch.set_num_threads(1);sets=prepare(folder);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    report={};rows=[];baselines=json.loads((Path(models)/'baselines.json').read_text(encoding='utf-8'))
    for split,values in sets.items():
        if split=='train':continue
        report[split]={}
        candidates=[]
        for model in sorted(Path(models).iterdir()):
            if not (model/'model.pt').exists():continue
            net,meta=load(model);prob,_=predict(net,meta,values['x']);_,quality=predict(net,meta,values['qx'])
            q=(meta.get('conformal') or {}).get('q')
            candidates.append((model.name,prob.argmax(1),quality,q))
        vz=values['qx'][:,-1,3]
        candidates += [('speed_quadratic_rules',rules(values['x']),np.c_[np.ones(len(vz)),abs(vz),vz*vz]@baselines['speed_quadratic'],None),
                       ('train_mean_rules',rules(values['x']),np.repeat(baselines['mean'],len(vz)),None)]
        for name,phase,quality,q in candidates:
            item=metrics(values['y'],phase);error=quality-values['qy']
            item.update(impact_mae=float(np.mean(abs(error))),impact_rmse=float(np.sqrt(np.mean(error**2))),
                        episodes=len(values['qy']),phase_windows=len(phase))
            if q is not None:
                item.update(interval_q_body_weights=float(q),interval_width_body_weights=2*float(q),
                            interval_coverage=float(np.mean(np.abs(error)<=q)),
                            interval_target_coverage=1-ALPHA)
            report[split][name]=item
            for ep,true,pred in zip(values['qepisode'],values['qy'],quality):
                rows.append([split,name,int(ep),float(true),float(pred),float(q) if q is not None else ''])
        if 'mlp_42' in report[split]:
            net,meta=load(Path(models)/'mlp_42');phase,_=predict(net,meta,values['x'])
            np.savez_compressed(out/(split+'_phase_predictions.npz'),y=values['y'],pred=phase.argmax(1),episode=values['episode'])
    (out/'metrics.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    with (out/'impact_predictions.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['split','model','episode','true_peak_load_factor','predicted_peak_load_factor','interval_half_width_body_weights']);writer.writerows(rows)
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['train','evaluate']);p.add_argument('--data',default='data/raw');p.add_argument('--models',default='models/trained');p.add_argument('--output',default='results');p.add_argument('--epochs',type=int,default=100)
    a=p.parse_args()
    if a.command=='train':train(a.data,a.models,a.epochs)
    else:evaluate(a.data,a.models,a.output)
