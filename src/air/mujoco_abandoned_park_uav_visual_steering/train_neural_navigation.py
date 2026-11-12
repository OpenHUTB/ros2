"""Train an A*-supervised MLP over generated occupancy maps."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random

import torch
from torch import nn

from navigation import OccupancyGrid, astar
from neural_navigation import LocalPolicy, NeuralPlanner


def generate_examples(map_count: int, seed: int) -> tuple[torch.Tensor, torch.Tensor]:
    """Use independent reachable maps and A* first steps as supervision."""
    rng = random.Random(seed)
    features: list[tuple[float, ...]] = []
    targets: list[int] = []
    size = 9
    attempts = 0
    while len(features) < map_count * 3 and attempts < map_count * 30:
        attempts += 1
        start = (rng.randrange(1, size - 1), rng.randrange(1, size - 1))
        goal = (rng.randrange(1, size - 1), rng.randrange(1, size - 1))
        if start == goal:
            continue
        grid = OccupancyGrid(width=size, height=size)
        grid.occupied = {
            (x, y) for x in range(1, size - 1) for y in range(1, size - 1)
            if (x, y) not in (start, goal) and rng.random() < 0.19
        }
        path = astar(grid, start, goal)
        if len(path) < 2:
            continue
        step = (path[1][0] - start[0], path[1][1] - start[1])
        direction_yaw = math.atan2(step[1], step[0])
        for label, index in (("LEFT", 0), ("STRAIGHT", 1), ("RIGHT", 2)):
            yaw = direction_yaw + (math.pi / 2 if label == "RIGHT" else -math.pi / 2 if label == "LEFT" else 0.0)
            vector = (math.cos(yaw), math.sin(yaw))
            side = (-vector[1], vector[0])

            def is_free(dx: float, dy: float) -> float:
                cell = (start[0] + round(dx), start[1] + round(dy))
                return float(grid.in_bounds(cell) and cell not in grid.occupied)

            gx, gy = goal[0] - start[0], goal[1] - start[1]
            distance = math.hypot(gx, gy)
            gx, gy = gx / distance, gy / distance
            features.append((
                is_free(*vector), is_free(*side), is_free(-side[0], -side[1]),
                gx * vector[0] + gy * vector[1], gx * side[0] + gy * side[1],
            ))
            targets.append(index)
    if len(features) != map_count * 3:
        raise RuntimeError(f"could create only {len(features) // 3} of {map_count} reachable maps")
    return torch.tensor(features, dtype=torch.float32), torch.tensor(targets, dtype=torch.long)


def train(
    output: Path,
    epochs: int = 24,
    seed: int = 250102,
    train_maps: int = 1200,
    test_maps: int = 300,
) -> dict[str, object]:
    random.seed(seed)
    torch.manual_seed(seed)
    train_x, train_y = generate_examples(train_maps, seed)
    test_x, test_y = generate_examples(test_maps, seed + 1)
    model = LocalPolicy()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.012)
    criterion = nn.CrossEntropyLoss()
    for _ in range(epochs):
        model.train()
        order = torch.randperm(len(train_y))
        for batch in order.split(128):
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(train_x[batch]), train_y[batch])
            loss.backward()
            optimizer.step()
    model.eval()
    with torch.inference_mode():
        train_accuracy = float((model(train_x).argmax(1) == train_y).float().mean())
        test_accuracy = float((model(test_x).argmax(1) == test_y).float().mean())
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "version": 1,
        "labels": NeuralPlanner.labels,
        "feature_names": NeuralPlanner.feature_names,
        "hidden_size": 16,
        "state_dict": model.state_dict(),
        "seed": seed,
    }, output)
    return {
        "seed": seed, "epochs": epochs, "train_maps": train_maps, "test_maps": test_maps,
        "train_samples": len(train_y), "test_samples": len(test_y),
        "train_accuracy": train_accuracy, "test_accuracy": test_accuracy,
        "checkpoint_bytes": output.stat().st_size,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Train occupancy-grid neural planner / 训练占用栅格神经规划器")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "models" / "weights" / "neural_navigation.pt")
    parser.add_argument("--epochs", type=int, default=24)
    parser.add_argument("--seed", type=int, default=250102)
    parser.add_argument("--train-maps", type=int, default=1200)
    parser.add_argument("--test-maps", type=int, default=300)
    args = parser.parse_args()
    print(json.dumps(train(args.output, args.epochs, args.seed, args.train_maps, args.test_maps), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
