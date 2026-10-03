"""Select weights and rejection threshold on validation flights only."""
import argparse
import json
import random
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader,TensorDataset
from pattern_model import PatternNet
from pattern_data import CLASSES,FEATURES


def train(output,variant,seed,epochs):
    if (output/'model.pt').exists():raise FileExistsError('Use a fresh output directory')
    torch.set_num_threads(3);torch.manual_seed(seed);np.random.seed(seed);random.seed(seed)
    data=np.load('data/prepared/train.npz');val=np.load('data/prepared/validation.npz')
    assert not set(data['episodes'])&set(val['episodes'])
    mean=data['x'].mean((0,1));std=np.maximum(data['x'].std((0,1)),.02)
    tx=torch.from_numpy((data['x']-mean)/std);ty=torch.from_numpy(data['y']).long()
    vx=torch.from_numpy((val['x']-mean)/std);vy=torch.from_numpy(val['y']).long()
    loader=DataLoader(TensorDataset(tx,ty),batch_size=32,shuffle=True)
    model=PatternNet(variant);optim=torch.optim.Adam(model.parameters(),lr=.002,weight_decay=1e-4)
    loss_fn=torch.nn.CrossEntropyLoss();best=float('inf');stale=0;history=[]
    output.mkdir(parents=True,exist_ok=True)
    for epoch in range(epochs):
        model.train();total=0
        for x,y in loader:
            optim.zero_grad();loss=loss_fn(model(x),y);loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),1.);optim.step();total+=loss.item()*len(x)
        model.eval()
        with torch.no_grad():vl=float(loss_fn(model(vx),vy))
        history.append({'epoch':epoch+1,'train_loss':total/len(tx),'validation_loss':vl})
        if vl<best-1e-5:
            best=vl;stale=0;torch.save(model.state_dict(),output/'model.pt')
        else:stale+=1
        if stale>=20:break
    model.load_state_dict(torch.load(output/'model.pt',weights_only=True));model.eval()
    with torch.no_grad():p=torch.softmax(model(vx),1).numpy()
    confidence=p.max(1);correct=p.argmax(1)==val['y']
    # Fixed policy: widest validation coverage that reaches 90% accepted accuracy and >=20% coverage.
    candidates=[]
    for threshold in np.linspace(.2,.95,76):
        accepted=confidence>=threshold
        if accepted.mean()>=.2 and correct[accepted].mean()>=.9:candidates.append(float(threshold))
    threshold=min(candidates) if candidates else .8
    config={'variant':variant,'seed':seed,'classes':CLASSES,'features':FEATURES,'mean':mean.tolist(),'std':std.tolist(),
            'confidence_threshold':threshold,'rejection_policy':'validation accuracy >=0.90 at maximum coverage >=0.20; fallback 0.80',
            'best_validation_loss':best,'history_s':6.,'warmup_s':7.,'inference_period_s':.5,'stable_predictions':3}
    (output/'config.json').write_text(json.dumps(config,indent=2))
    (output/'loss.json').write_text(json.dumps(history,indent=2))
    print(json.dumps({'variant':variant,'seed':seed,'val_loss':best,'threshold':threshold,'epochs':len(history)}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path('models/retrained_gru_42'))
    p.add_argument('--variant',choices=['gru','mlp','position_only','mlp_position_only'],default='mlp');p.add_argument('--seed',type=int,default=42)
    p.add_argument('--epochs',type=int,default=120);a=p.parse_args();train(a.output,a.variant,a.seed,a.epochs)
