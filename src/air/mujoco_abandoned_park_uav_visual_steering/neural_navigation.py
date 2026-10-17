"""Neural-network local planning coupled to the incremental SLAM map."""
from __future__ import annotations
from dataclasses import dataclass
import math
from navigation import Cell, OccupancyGrid

@dataclass(frozen=True)
class NeuralAction:
    linear: float
    angular: float
    label: str

class NeuralPlanner:
    """Deterministic MLP policy; A* remains only a baseline."""
    labels = ("LEFT", "STRAIGHT", "RIGHT")
    def __init__(self) -> None:
        self.w1 = ((1.2,-0.8,0.8,0.4,0.2),(-0.5,1.1,-1.1,0.2,0.4),(0.7,0.7,0.7,0.8,0.8),(-0.6,-0.4,-0.4,0.3,0.3))
        self.w2 = ((-1.0,1.0,0.2,-0.1),(0.3,0.8,0.2,0.1),(1.0,-0.8,0.2,-0.1))
    def predict(self, grid: OccupancyGrid, current: Cell, goal: Cell) -> NeuralAction:
        def free(dx: int, dy: int) -> float:
            c=(current[0]+dx,current[1]+dy)
            return 0.0 if c in grid.occupied or not grid.in_bounds(c) else 1.0
        features=(free(0,1),free(-1,0),free(1,0),max(-1,min(1,(goal[0]-current[0])/max(1,grid.width))),max(-1,min(1,(goal[1]-current[1])/max(1,grid.height))))
        hidden=tuple(math.tanh(sum(w*x for w,x in zip(row,features))) for row in self.w1)
        logits=tuple(sum(w*x for w,x in zip(row,hidden)) for row in self.w2)
        label=self.labels[max(range(3),key=logits.__getitem__)]
        return NeuralAction(0.6, {"LEFT":-0.8,"STRAIGHT":0.0,"RIGHT":0.8}[label], label)
