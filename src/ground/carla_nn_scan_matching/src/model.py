"""1D CNN 雷达扫描匹配模型
输入：两帧连续的 2D 雷达扫描数据 (Batch, 2, 360)
输出：两帧之间的相对位姿 (dx, dy, dtheta)
"""
import torch
import torch.nn as nn

class ScanMatcherCNN(nn.Module):
    def __init__(self, scan_size=360):
        super(ScanMatcherCNN, self).__init__()
        
        # 1D 卷积层提取局部特征
        self.conv1 = nn.Conv1d(in_channels=2, out_channels=16, kernel_size=5, padding=2)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool1d(kernel_size=2)
        
        self.conv2 = nn.Conv1d(in_channels=16, out_channels=32, kernel_size=5, padding=2)
        self.relu2 = nn.ReLU()
        self.pool2 = nn.MaxPool1d(kernel_size=2)
        
        self.flatten = nn.Flatten()
        
        # 计算全连接层输入维度 (360 -> 180 -> 90)
        fc_input_dim = 32 * (scan_size // 4)
        
        self.fc1 = nn.Linear(fc_input_dim, 128)
        self.relu3 = nn.ReLU()
        self.dropout = nn.Dropout(0.2)
        
        # 输出 3 个值：dx, dy, dtheta
        self.fc2 = nn.Linear(128, 3)
        
    def forward(self, x):
        # x shape: (Batch, 2, 360)
        x = self.pool1(self.relu1(self.conv1(x)))
        x = self.pool2(self.relu2(self.conv2(x)))
        x = self.flatten(x)
        x = self.dropout(self.relu3(self.fc1(x)))
        out = self.fc2(x)
        return out

if __name__ == '__main__':
    model = ScanMatcherCNN()
    total_params = sum(p.numel() for p in model.parameters())
    print("ScanMatcherCNN 参数量:", total_params)
    
    # 测试前向传播
    dummy_input = torch.randn(1, 2, 360)
    out = model(dummy_input)
    print("输入 shape:", dummy_input.shape)
    print("输出 shape:", out.shape)
    print("输出示例 (dx, dy, dtheta):", out[0].tolist())
