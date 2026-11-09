import torch
import torch.nn as nn
from torchvision import datasets, transforms
from torch.utils.data import DataLoader

class SonarCNN(nn.Module):
    def __init__(self, num_classes=4):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 28 * 28, 128),
            nn.ReLU(),
            nn.Linear(128, num_classes)
        )
    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

def main():
    transform = transforms.Compose([
        transforms.Resize((224,224)),
        transforms.ToTensor()
    ])
    # 数据集路径，你后续把声呐图片数据集放到这个文件夹
    dataset = datasets.ImageFolder(root="./sonar_dataset", transform=transform)
    loader = DataLoader(dataset, batch_size=4, shuffle=True)

    model = SonarCNN(num_classes=4)
    loss_fn = nn.CrossEntropyLoss()
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)

    for epoch in range(6):
        total_loss = 0.0
        for img, label in loader:
            pred = model(img)
            loss = loss_fn(pred, label)
            opt.zero_grad()
            loss.backward()
            opt.step()
            total_loss += loss.item()
        print(f"Epoch {epoch+1}, loss = {total_loss:.4f}")
    # 保存权重
    torch.save(model.state_dict(), "sonar_cnn_weights.pth")
    print("权重文件 sonar_cnn_weights.pth 生成完成！")

if __name__ == "__main__":
    main()
