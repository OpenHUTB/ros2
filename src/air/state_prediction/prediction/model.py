import numpy as np
import torch
from torch import nn
from .data import baseline


class Predictor(nn.Module):
    def __init__(self, variant='gru', hidden=48):
        super().__init__()
        self.variant=variant
        self.columns=list(range(6)) if variant=='no_acc_att' else list(range(16))
        if variant=='mlp':
            self.encoder=nn.Sequential(nn.Linear(len(self.columns),hidden),nn.Tanh(),nn.Linear(hidden,hidden),nn.Tanh())
        else:
            self.encoder=nn.GRU(len(self.columns),hidden,batch_first=True)
        self.head=nn.Linear(hidden,18)
        nn.init.zeros_(self.head.weight); nn.init.zeros_(self.head.bias)

    def forward(self,x):
        x=x[:,:,self.columns]
        if self.variant=='mlp': z=self.encoder(x[:,-1])
        else: _,h=self.encoder(x); z=h[-1]
        return self.head(z).reshape(-1,3,6)


def load_predictor(path):
    # Only load locally trained checkpoints. weights_only avoids arbitrary object loading.
    ckpt=torch.load(path,map_location='cpu',weights_only=True)
    model=Predictor(ckpt['variant'],ckpt['hidden']); model.load_state_dict(ckpt['state_dict']); model.eval()
    return model,ckpt


def predict(model,ckpt,x,current):
    with torch.no_grad():
        normal=(torch.as_tensor(x,dtype=torch.float32)-ckpt['x_mean'])/ckpt['x_std']
        residual=model(normal)*ckpt['y_std']+ckpt['y_mean']
    return baseline(current)+residual.numpy()
