"""Reproducible simulator data and held-out-scene evaluation."""
import json
from pathlib import Path
import time

import mujoco
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from .scene import IndoorSimulator
from .vision import FloorNet, ImageController, MaskController, mask_command


def collect(assets, destination, samples=100):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    for split, seeds in [('train', range(6)), ('validation', [6]), ('test', [7, 8])]:
        images, masks, actions, scene_ids = [], [], [], []
        for seed in seeds:
            sim = IndoorSimulator(assets, seed)
            rng = np.random.default_rng(1000+seed)
            try:
                for _ in range(samples):
                    # Independent static camera sampling, NOT a runtime walking method.
                    mujoco.mj_resetData(sim.model, sim.data)
                    sim.data.qpos[:2] = [rng.uniform(.0, 8.8), rng.uniform(-.9, .9)]
                    yaw = rng.uniform(-.65, .65)
                    sim.data.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
                    sim.data.qpos[7:] = sim.default
                    mujoco.mj_forward(sim.model, sim.data)
                    rgb, mask = sim.image(labels=True)
                    images.append(rgb)
                    masks.append(mask)
                    actions.append(mask_command(mask))
                    scene_ids.append(seed)
            finally:
                sim.close()
            print(f'collected {split} scene={seed}', flush=True)
        np.savez_compressed(destination/f'{split}.npz', images=np.asarray(images),
            masks=np.asarray(masks), actions=np.asarray(actions), scene_ids=scene_ids)
    metadata = dict(split_by='scene_seed', train=list(range(6)), validation=[6], test=[7,8],
        samples_per_scene=samples, labels='MuJoCo geometry-ID floor masks',
        teacher='reactive floor-mask corridor following',
        limitations='Static camera poses; appearance/layout randomization is limited. Closed-loop testing required.')
    (destination/'dataset.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')


def train(dataset, output, epochs=15):
    torch.manual_seed(42)
    torch.set_num_threads(4)
    dataset, output = Path(dataset), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    def read(split):
        with np.load(dataset/f'{split}.npz') as d:
            return (torch.from_numpy(d['images']).permute(0,3,1,2).float()/255,
                    torch.from_numpy(d['masks']).float()[:,None],
                    torch.from_numpy(d['actions']).float())
    train_data, val_data, test_data = read('train'), read('validation'), read('test')
    metrics = {}
    for mode, model, idx in [('perception', FloorNet(), 1)]:
        loader = DataLoader(TensorDataset(train_data[0],train_data[idx]), batch_size=16, shuffle=True)
        optimizer = torch.optim.Adam(model.parameters(), lr=.001)
        loss_fn = nn.BCEWithLogitsLoss() if idx == 1 else nn.MSELoss()
        best = float('inf')
        history = []
        start = time.time()
        for epoch in range(epochs):
            model.train()
            total = 0.
            for x,y in loader:
                optimizer.zero_grad()
                loss = loss_fn(model(x),y)
                loss.backward()
                optimizer.step()
                total += float(loss.detach())*len(x)
            model.eval()
            with torch.inference_mode():
                val = float(loss_fn(model(val_data[0]),val_data[idx]))
            history.append(dict(epoch=epoch+1, train_loss=total/len(train_data[0]), validation_loss=val))
            if val < best:
                best = val
                torch.save(model.state_dict(),output/f'{mode}.pt')
            print(mode,history[-1],flush=True)
        model.load_state_dict(torch.load(output/f'{mode}.pt',weights_only=True))
        with torch.inference_mode():
            pred = model(test_data[0])
            if idx == 1:
                pred = pred.sigmoid()>.5
                truth = test_data[idx]>.5
                score = dict(floor_iou=float((pred & truth).sum()/(pred | truth).sum().clamp(min=1)))
            else:
                error = (pred-test_data[idx]).abs().mean(0)
                score = dict(speed_mae_mps=float(error[0]), yaw_rate_mae_radps=float(error[1]))
        metrics[mode] = dict(test=score, history=history, training_seconds=time.time()-start)
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    return metrics


def train_mask_controller(dataset, output, epochs=100):
    """Train on scene-disjoint semantic masks; test only after model selection."""
    torch.manual_seed(73)
    torch.set_num_threads(2)
    dataset,output=Path(dataset),Path(output)
    def load(split):
        with np.load(dataset/f'{split}.npz') as d:
            return torch.from_numpy(d['masks']).float()[:,None],torch.from_numpy(d['actions'])
    x,y=load('train')
    xv,yv=load('validation')
    xt,yt=load('test')
    model=MaskController()
    optimizer=torch.optim.Adam(model.parameters(),lr=.002)
    loader=DataLoader(TensorDataset(x,y),batch_size=32,shuffle=True)
    best=float('inf')
    for epoch in range(epochs):
        model.train()
        for xb,yb in loader:
            loss=nn.functional.mse_loss(model(xb),yb)
            optimizer.zero_grad(); loss.backward(); optimizer.step()
        model.eval()
        with torch.inference_mode():
            val=float(nn.functional.mse_loss(model(xv),yv))
        if val<best:
            best=val
            torch.save(model.state_dict(),output/'mask_controller.pt')
    model.load_state_dict(torch.load(output/'mask_controller.pt',weights_only=True))
    with torch.inference_mode():
        mae=(model(xt)-yt).abs().mean(0).tolist()
    metrics=json.loads((output/'metrics.json').read_text())
    metrics['mask_controller']=dict(epochs=epochs,validation_mse=best,
        test_speed_mae_mps=mae[0],test_yaw_rate_mae_radps=mae[1])
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    print(metrics['mask_controller'],flush=True)
