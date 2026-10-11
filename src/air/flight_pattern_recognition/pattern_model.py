import torch
from torch import nn


class PatternNet(nn.Module):
    def __init__(self,variant='gru'):
        super().__init__();self.variant=variant
        if variant in ('gru','position_only'):
            self.encoder=nn.GRU(8,32,batch_first=True)
            self.head=nn.Sequential(nn.Linear(32,24),nn.ReLU(),nn.Linear(24,5))
        else:
            self.head=nn.Sequential(nn.Linear(16,32),nn.ReLU(),nn.Linear(32,5))

    def forward(self,x):
        if self.variant in ('position_only','mlp_position_only'):
            x=x.clone();x[:,:,:5]=0
        if self.variant in ('gru','position_only'):
            _,h=self.encoder(x);return self.head(h[-1])
        return self.head(torch.cat([x.mean(1),x.std(1,unbiased=False)],dim=1))
