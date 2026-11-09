import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms
import os
from PIL import Image

class SonarCNN(nn.Module):
    def __init__(self, num_classes=3):
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
            nn.Linear(64 * 32 * 32, 128),
            nn.ReLU(),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

class DummySonarDataset(Dataset):
    def __init__(self, length=120):
        self.length = length
        self.transform = transforms.Compose([
            transforms.Resize((256,256)),
            transforms.ToTensor()
        ])
    def __len__(self):
        return self.length
    def __getitem__(self, idx):
        rand_img = torch.randn(3,256,256).clamp(0,1)
        label = idx % 3
        return rand_img, label

def main():
    device = torch.device("cpu")
    model = SonarCNN(num_classes=3).to(device)
    dataset = DummySonarDataset()
    loader = DataLoader(dataset, batch_size=8, shuffle=True)

    criterion = nn.CrossEntropyLoss()
    opt = optim.Adam(model.parameters(), lr=1e-3)

    epoch_num = 10
    for ep in range(epoch_num):
        total_loss = 0.0
        for img, label in loader:
            img, label = img.to(device), label.to(device)
            pred = model(img)
            loss = criterion(pred, label)
            opt.zero_grad()
            loss.backward()
            opt.step()
            total_loss += loss.item()
        print(f"Epoch {ep+1}, Loss: {total_loss:.4f}")

    torch.save(model.state_dict(), "sonar_cnn_weights.pth")
    print("训练完成，权重已保存至 sonar_cnn_weights.pth")

if __name__ == "__main__":
    main()
