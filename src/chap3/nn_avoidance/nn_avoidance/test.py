import torch
import torch.nn as nn

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
    model.load_state_dict(torch.load("model.pth"))
    model.eval()
    test_input = torch.randn(1,8)
    out = model(test_input)
    print("网络输出左右轮速度：", out.detach().numpy())
    print("测试完成")