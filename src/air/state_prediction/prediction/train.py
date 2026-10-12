import argparse
import csv
import hashlib
import json
import random
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import TensorDataset,DataLoader
from .data import baseline
from .model import Predictor


def train(data_dir,out,variant,seed,epochs=45):
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    torch.set_num_threads(3); torch.use_deterministic_algorithms(True)
    out.mkdir(parents=True,exist_ok=True)
    train_set=np.load(data_dir/'train.npz'); val=np.load(data_dir/'validation.npz')
    x=torch.from_numpy(train_set['x'])
    y=torch.from_numpy(train_set['truth']-baseline(train_set['current']))
    mean=x.mean((0,1)); std=x.std((0,1)).clamp_min(0.01)
    ym=y.mean(0); ys=y.std(0).clamp_min(0.01)
    dataset=TensorDataset((x-mean)/std,(y-ym)/ys)
    loader=DataLoader(dataset,batch_size=128,shuffle=True,generator=torch.Generator().manual_seed(seed))
    vx=(torch.from_numpy(val['x'])-mean)/std
    vy=(torch.from_numpy(val['truth']-baseline(val['current']))-ym)/ys
    model=Predictor(variant); optimizer=torch.optim.Adam(model.parameters(),lr=0.002,weight_decay=1e-5)
    loss_fn=torch.nn.MSELoss(); best=float('inf'); stale=0; history=[]
    manifest_hash=hashlib.sha256((data_dir/'manifest.json').read_bytes()).hexdigest()
    for epoch in range(1,epochs+1):
        model.train(); total=0
        for bx,by in loader:
            optimizer.zero_grad(); loss=loss_fn(model(bx),by); loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),1.0); optimizer.step()
            total+=loss.item()*len(bx)
        model.eval()
        with torch.no_grad(): val_loss=loss_fn(model(vx),vy).item()
        history.append([epoch,total/len(dataset),val_loss])
        if val_loss < best-1e-5:
            best=val_loss; stale=0
            torch.save(dict(state_dict=model.state_dict(),variant=variant,hidden=48,seed=seed,epoch=epoch,
                x_mean=mean,x_std=std,y_mean=ym,y_std=ys,validation_loss=best,manifest_sha256=manifest_hash),out/'model.pt')
        else: stale+=1
        if stale>=8: break
    with (out/'loss.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['epoch','train_normalized_mse','validation_normalized_mse']); w.writerows(history)
    (out/'config.json').write_text(json.dumps(dict(variant=variant,seed=seed,max_epochs=epochs,best_validation_loss=best,
        optimizer='Adam',learning_rate=0.002,batch_size=128,early_stopping_patience=8,manifest_sha256=manifest_hash),indent=2))
    print(variant,seed,'best_validation_loss',best,flush=True)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--data',type=Path,default=Path('data/processed')); p.add_argument('--output',type=Path,default=Path('models'))
    p.add_argument('--seeds',type=int,nargs='+',default=[42,43,44]); p.add_argument('--epochs',type=int,default=45)
    p.add_argument('--variants',nargs='+',default=['gru','no_acc_att','mlp']); a=p.parse_args()
    for variant in a.variants:
        for seed in a.seeds: train(a.data,a.output/(variant+'_'+str(seed)),variant,seed,a.epochs)


if __name__=='__main__': main()
