import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

class AvoidNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(8,32),
            nn.ReLU(),
            nn.Linear(32,2)
        )
    def forward(self,x):
        return self.net(x)

if __name__=="__main__":
    model = AvoidNet()
    train_x = np.random.rand(1000,8).astype(np.float32)
    train_y = np.random.rand(1000,2).astype(np.float32)
    x = torch.from_numpy(train_x)
    y = torch.from_numpy(train_y)

    loss_fn = nn.MSELoss()
    opt = optim.Adam(model.parameters(), lr=1e-3)

    for epoch in range(300):
        pred = model(x)
        loss = loss_fn(pred,y)
        opt.zero_grad()
        loss.backward()
        opt.step()
        if epoch%50 ==0:
            print(f"epoch:{epoch}, loss:{loss.item():.4f}")
    torch.save(model.state_dict(),"model.pth")
    print("训练完成，模型保存 model.pth")