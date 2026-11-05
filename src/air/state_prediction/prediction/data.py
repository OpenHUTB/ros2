"""Episode-level splits, uniform-time resampling and leakage-free windows."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

DT = 0.05
HISTORY = 20
HORIZONS = np.array([0.25, 0.5, 1.0], dtype=np.float32)
OFFSETS = np.array([5, 10, 20])
STATE_FIELDS = ['px','py','pz','vx','vy','vz','ax','ay','az','qw','qx','qy','qz','wx','wy','wz']


def read_episode(path):
    with Path(path).open() as f:
        rows = list(csv.DictReader(f))
    stamps = np.array([int(r['timestamp_ns']) for r in rows], dtype=np.int64)
    states = np.array([[float(r[k]) for k in STATE_FIELDS] for r in rows])
    if len(stamps) < HISTORY+OFFSETS[-1]+2:
        raise ValueError('Episode too short: '+str(path))
    if not np.isfinite(states).all():
        raise ValueError('Nonfinite state: '+str(path))
    if any(int(r['collision']) for r in rows):
        raise ValueError('Collision in episode: '+str(path))
    delta = np.diff(stamps)*1e-9
    if np.any(delta <= 0) or delta.max() > 0.15:
        raise ValueError('Nonmonotonic timestamps or gap > 150 ms: '+str(path))
    t = (stamps-stamps[0])*1e-9
    # Quaternion sign is arbitrary: choose a continuous representation before interpolation.
    for i in range(1,len(states)):
        if np.dot(states[i,9:13],states[i-1,9:13]) < 0:
            states[i,9:13] *= -1
    grid=np.arange(0,t[-1]+1e-9,DT)
    uniform=np.column_stack([np.interp(grid,t,states[:,i]) for i in range(states.shape[1])])
    norm=np.linalg.norm(uniform[:,9:13],axis=1,keepdims=True)
    if np.any(norm<0.5):
        raise ValueError('Invalid orientation quaternion')
    uniform[:,9:13] /= norm
    return grid,uniform.astype(np.float32)


def features(history):
    result=np.array(history,dtype=np.float32,copy=True)
    result[...,:3] -= result[...,-1:,:3]
    return result


def baseline(current, acceleration=False):
    p=current[...,:3]; v=current[...,3:6]; a=current[...,6:9]
    h=HORIZONS.reshape((-1,1))
    pos=p[...,None,:]+v[...,None,:]*h
    vel=np.broadcast_to(v[...,None,:],pos.shape).copy()
    if acceleration:
        pos=pos+0.5*a[...,None,:]*h*h
        vel=vel+a[...,None,:]*h
    return np.concatenate([pos,vel],axis=-1)


def build(raw, output):
    output.mkdir(parents=True,exist_ok=True)
    splits={k:[] for k in ['train','validation','test']}
    manifest=[]
    for path in sorted(raw.glob('episode_*.csv')):
        meta=json.loads(path.with_suffix('.json').read_text())
        checksum=hashlib.sha256(path.read_bytes()).hexdigest()
        if checksum != meta['sha256']:
            raise ValueError('Checksum mismatch: '+str(path))
        grid,state=read_episode(path)
        split=meta['split']
        if split not in splits:
            raise ValueError('Unknown split')
        # Every window and all its labels belong to this single episode.
        for index in range(HISTORY-1,len(state)-OFFSETS[-1]):
            hist=state[index-HISTORY+1:index+1]
            truth=state[index+OFFSETS,:6]
            splits[split].append((features(hist),truth,state[index],meta['episode_id'],grid[index]))
        manifest.append(dict(episode_id=meta['episode_id'],split=split,family=meta['family'],sha256=checksum,samples=len(state)))
    for split,items in splits.items():
        if not items:
            raise ValueError('Missing split: '+split)
        x,y,current,episode,t=zip(*items)
        np.savez_compressed(output/(split+'.npz'),x=np.array(x),truth=np.array(y),current=np.array(current),episode=np.array(episode),t=np.array(t))
    (output/'manifest.json').write_text(json.dumps(dict(dt=DT,history=HISTORY,horizons=HORIZONS.tolist(),episodes=manifest),indent=2))
    print({split:len(items) for split,items in splits.items()})


def main():
    p=argparse.ArgumentParser(); p.add_argument('--raw',type=Path,default=Path('data/raw')); p.add_argument('--output',type=Path,default=Path('data/processed'))
    a=p.parse_args(); build(a.raw,a.output)


if __name__=='__main__': main()
