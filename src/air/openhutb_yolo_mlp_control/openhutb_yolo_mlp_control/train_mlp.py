import argparse

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split

from .mlp_model import DEFAULT_INPUT_SCALE, DEFAULT_OUTPUT_SCALE, MLPController


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='controller_dataset.csv')
    parser.add_argument('--model', default='mlp_controller.pth')
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--batch-size', type=int, default=256)
    parser.add_argument('--lr', type=float, default=0.001)
    parsed = parser.parse_args(args)

    data = np.loadtxt(parsed.dataset, delimiter=',', skiprows=1, dtype=np.float32)
    x = torch.tensor(data[:, 0:6] / DEFAULT_INPUT_SCALE, dtype=torch.float32)
    y = torch.tensor(data[:, 6:9] / DEFAULT_OUTPUT_SCALE, dtype=torch.float32)
    dataset = TensorDataset(x, y)
    train_size = int(len(dataset) * 0.8)
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    train_loader = DataLoader(train_dataset, batch_size=parsed.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=parsed.batch_size)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('训练设备:', device)
    if torch.cuda.is_available():
        print('GPU:', torch.cuda.get_device_name(0))

    model = MLPController().to(device)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=parsed.lr)

    for epoch in range(1, parsed.epochs + 1):
        model.train()
        train_loss = 0.0
        for xb, yb in train_loader:
            xb, yb = xb.to(device), yb.to(device)
            optimizer.zero_grad()
            output = model(xb)
            loss = criterion(output, yb)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * xb.size(0)
        train_loss /= len(train_dataset)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                val_loss += criterion(model(xb), yb).item() * xb.size(0)
        val_loss /= len(val_dataset)

        if epoch == 1 or epoch % 5 == 0:
            print(f'Epoch {epoch:3d}/{parsed.epochs} Train={train_loss:.6f} Val={val_loss:.6f}')

    torch.save({
        'model_state_dict': model.state_dict(),
        'input_scale': DEFAULT_INPUT_SCALE,
        'output_scale': DEFAULT_OUTPUT_SCALE,
    }, parsed.model)
    print('训练完成')
    print('模型:', parsed.model)


if __name__ == '__main__':
    main()
