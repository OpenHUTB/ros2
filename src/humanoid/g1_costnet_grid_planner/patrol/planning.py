"""Learned cost-to-go tie breaker for admissible grid A*.

Manhattan f is the primary priority, so learned estimates cannot invalidate
optimality on the four-connected grid. The network resolves equal-f plateaus.
This is neural-assisted search, not an end-to-end replacement of graph search.
"""
import heapq
import json
from pathlib import Path
from functools import lru_cache
import math

import numpy as np
import torch
from torch import nn


class CostNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net=nn.Sequential(nn.Conv2d(3,16,3,padding=1),nn.ReLU(),
            nn.Conv2d(16,24,3,padding=2,dilation=2),nn.ReLU(),
            nn.Conv2d(24,24,3,padding=4,dilation=4),nn.ReLU(),
            nn.Conv2d(24,1,1),nn.Softplus())

    def forward(self,x):
        return self.net(x)


def features(grid,goal):
    yy,xx=np.indices(grid.shape)
    height, width = grid.shape
    return np.stack([grid, (xx - goal[1]) / max(32, width),
                     (yy - goal[0]) / max(32, height)]).astype(np.float32)


def distances(grid,goal):
    result=np.full(grid.shape,np.inf,dtype=np.float32)
    result[goal]=0
    queue=[(0,goal)]
    while queue:
        cost,point=heapq.heappop(queue)
        if cost!=result[point]: continue
        for dy,dx in [(1,0),(-1,0),(0,1),(0,-1)]:
            q=(point[0]+dy,point[1]+dx)
            if 0<=q[0]<grid.shape[0] and 0<=q[1]<grid.shape[1] and not grid[q] and cost+1<result[q]:
                result[q]=cost+1
                heapq.heappush(queue,(cost+1,q))
    return result


def train_planner(output, epochs=20, sample_count=240, seed=91):
    torch.manual_seed(seed)
    torch.set_num_threads(3)
    def samples(seeds):
        xs,ys,masks=[],[],[]
        for seed in seeds:
            rng=np.random.default_rng(seed)
            grid=np.zeros((32,32),dtype=bool)
            grid[[0,-1],:]=True;grid[:,[0,-1]]=True
            for _ in range(8):
                y,x=rng.integers(2,27,size=2)
                h,w=rng.integers(1,5,size=2)
                grid[y:y+h,x:x+w]=True
            free=np.argwhere(~grid)
            goal=tuple(free[rng.integers(len(free))])
            d=distances(grid,goal)
            valid=np.isfinite(d)
            xs.append(features(grid,goal));ys.append(np.where(valid,d,0)/32.)
            masks.append(valid)
        return torch.from_numpy(np.asarray(xs)),torch.from_numpy(np.asarray(ys))[:,None],torch.from_numpy(np.asarray(masks))[:,None]
    first_train_seed = (seed - 91) % 2**32
    x,y,mask=samples(range(first_train_seed, first_train_seed + sample_count))
    first_eval_seed = max(1000, first_train_seed + sample_count)
    xv,yv,mv=samples(range(first_eval_seed, first_eval_seed + 40))
    first_test_seed = max(2000, first_eval_seed + 40)
    xt,yt,mt=samples(range(first_test_seed, first_test_seed + 40))
    model=CostNet()
    opt=torch.optim.Adam(model.parameters(),lr=.001)
    best=float('inf');history=[]
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    for epoch in range(epochs):
        for ids in torch.randperm(len(x)).split(16):
            loss=((model(x[ids])-y[ids])**2)[mask[ids]].mean()
            opt.zero_grad();loss.backward();opt.step()
        with torch.inference_mode(): val=float(((model(xv)-yv)**2)[mv].mean())
        history.append(val)
        if val<best:
            best=val;torch.save(model.state_dict(),output/'planner.pt')
    model.load_state_dict(torch.load(output/'planner.pt',weights_only=True))
    with torch.inference_mode(): mae=float((model(xt)-yt).abs()[mt].mean()*32)
    result=dict(train_maps=sample_count, epochs=epochs, seed=seed,validation_maps=40,test_maps=40,test_mae_cells=mae,
                validation_mse=history,scope='synthetic occupancy maps; not visual training scenes')
    (output/'planner_metrics.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


def astar(grid,start,goal,learned=None):
    if grid[start] or grid[goal]: raise ValueError('Start or goal is occupied/unknown')
    def h(p): return abs(p[0]-goal[0])+abs(p[1]-goal[1])
    queue=[(h(start),0,0,start)]
    best={start:0};parent={};closed=set();expanded=0
    while queue:
        _,_,g,p=heapq.heappop(queue)
        if p in closed: continue
        closed.add(p);expanded+=1
        if p==goal:
            route=[p]
            while p!=start: p=parent[p];route.append(p)
            return route[::-1],expanded
        for dy,dx in [(1,0),(-1,0),(0,1),(0,-1)]:
            q=(p[0]+dy,p[1]+dx)
            if not (0<=q[0]<grid.shape[0] and 0<=q[1]<grid.shape[1]) or grid[q]: continue
            cost=g+1
            if cost<best.get(q,float('inf')):
                best[q]=cost;parent[q]=p
                tie=float(learned[q]) if learned is not None else 0.
                heapq.heappush(queue,(cost+h(q),tie,cost,q))
    raise ValueError('No path in known free map')


@lru_cache(maxsize=2)
def load_model(path, revision):
    model = CostNet()
    model.load_state_dict(torch.load(path, map_location='cpu', weights_only=True))
    return model.eval()


def predict_costs(grid, goal, weights):
    path = (Path(weights) / 'planner.pt').resolve()
    model = load_model(path, path.stat().st_mtime_ns)
    with torch.inference_mode():
        return model(torch.from_numpy(features(grid, goal))[None])[0, 0].numpy()


def plan_request(request, weights, *, robot_radius=0.3, stride=3):
    original=np.asarray(request['map'],dtype=np.int16).reshape(request['height'],request['width'])
    resolution=float(request['resolution'])
    robot_radius = float(request.get('robot_radius', robot_radius))
    stride = request.get('stride', stride)
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be finite and positive')
    if not math.isfinite(robot_radius) or robot_radius < 0:
        raise ValueError('robot_radius must be finite and nonnegative')
    if type(stride) is not int or not 1 <= stride <= min(original.shape):
        raise ValueError('stride must be a positive integer within the map dimensions')
    # Exploration mode: inflate known obstacles; unknown space remains provisional.
    # Execution still uses the live local obstacle costmap. No safety claim is made
    # for unobserved portions of this global route.
    blocked=original>50
    radius=int(np.ceil(robot_radius/resolution))
    inflated=blocked.copy()
    for dy in range(-min(radius, len(blocked)-1), min(radius, len(blocked)-1)+1):
        for dx in range(-min(radius, blocked.shape[1]-1), min(radius, blocked.shape[1]-1)+1):
            if dx*dx+dy*dy>radius*radius: continue
            srcy=slice(max(0,-dy),min(len(blocked),len(blocked)-dy))
            srcx=slice(max(0,-dx),min(blocked.shape[1],blocked.shape[1]-dx))
            dsty=slice(max(0,dy),min(len(blocked),len(blocked)+dy))
            dstx=slice(max(0,dx),min(blocked.shape[1],blocked.shape[1]+dx))
            inflated[dsty,dstx]|=blocked[srcy,srcx]
    hh,ww=inflated.shape[0]//stride,inflated.shape[1]//stride
    grid=inflated[:hh*stride,:ww*stride].reshape(hh,stride,ww,stride).any(axis=(1,3))
    cell=resolution*stride;ox,oy=request['origin']
    def index(xy):
        p=(math.floor((xy[1]-oy)/cell), math.floor((xy[0]-ox)/cell))
        if not(0<=p[0]<hh and 0<=p[1]<ww):raise ValueError('Goal/start outside map')
        return p
    start,goal=index(request['start']),index(request['goal'])
    learned = predict_costs(grid, goal, weights)
    route,expanded=astar(grid,start,goal,learned)
    baseline,baseline_expanded=astar(grid,start,goal)
    # Keep corners and an occasional waypoint on long straight segments.
    keep=[route[0]]
    for i in range(1,len(route)-1):
        before=np.subtract(route[i],route[i-1]);after=np.subtract(route[i+1],route[i])
        if not np.array_equal(before,after) or np.linalg.norm(np.subtract(route[i],keep[-1]))*cell>=1.5:
            keep.append(route[i])
    keep.append(route[-1])
    world=[[ox+(p[1]+.5)*cell,oy+(p[0]+.5)*cell] for p in keep[1:]]
    world[-1]=request['goal']
    return dict(waypoints=world,expanded=expanded,baseline_expanded=baseline_expanded,
                path_cost=len(route)-1,baseline_path_cost=len(baseline)-1,
                unknown_policy='provisionally traversable; live local obstacle avoidance required',
                method='Manhattan A* with neural equal-f tie breaking')
