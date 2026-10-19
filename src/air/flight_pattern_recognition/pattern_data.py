"""Causal, planar-rotation-invariant flight pattern features from real telemetry."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

CLASSES = ['line', 'circle', 'figure_eight', 'climb', 'stop_go']
KEYS = ['px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']
HISTORY_S = 6.0
DT = .1
FEATURES = ['horizontal_speed', 'vertical_speed', 'tangential_accel', 'signed_normal_accel',
            'vertical_accel', 'horizontal_displacement', 'relative_height', 'path_efficiency']


def geometry(z):
    """z is sampled NED position/velocity/acceleration. No labels or command inputs."""
    v=z[:,3:5]; a=z[:,6:8]; speed=np.linalg.norm(v,axis=1)
    denom=np.maximum(speed,.05)
    tangential=(v*a).sum(1)/denom
    normal=(v[:,0]*a[:,1]-v[:,1]*a[:,0])/denom
    displacement=np.linalg.norm(z[:,:2]-z[0,:2],axis=1)
    length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(z[:,:2],axis=0),axis=1))]
    efficiency=np.divide(displacement,length,out=np.zeros_like(displacement),where=length>1e-6)
    return np.column_stack([speed,z[:,5],tangential,normal,z[:,8],displacement,z[:,2]-z[0,2],efficiency]).astype(np.float32)


def sample_window(stamps, values, end_ns):
    stamps=np.asarray(stamps,dtype=np.int64)
    end_ns=int(end_ns); start_ns=end_ns-int(HISTORY_S*1e9)
    # Inspect only the required history; future samples must never affect this result.
    lo=max(0,int(np.searchsorted(stamps,start_ns))-1)
    hi=int(np.searchsorted(stamps,end_ns,side='right'))
    t=stamps[lo:hi];z=np.asarray(values,dtype=np.float64)[lo:hi]
    if len(t)<2 or t[0]>start_ns or t[-1]<end_ns or np.any(np.diff(t)<=0) or np.any(np.diff(t)>150_000_000):
        return None
    if not np.isfinite(z).all(): return None
    relative=(t-end_ns)/1e9
    grid=np.linspace(-HISTORY_S,0,61)
    return geometry(np.column_stack([np.interp(grid,relative,z[:,k]) for k in range(9)]))


def read_flight(path):
    with Path(path).open(newline='') as f: rows=list(csv.DictReader(f))
    t=np.array([int(r['timestamp_ns']) for r in rows],np.int64)
    z=np.array([[float(r[k]) for k in KEYS] for r in rows])
    if len(t)<2 or not np.all(np.diff(t)>0) or not np.isfinite(z).all():
        raise ValueError('Invalid raw flight')
    return rows,t,z


def windows(path):
    rows,t,z=read_flight(path)
    # Ignore the first second of each collection episode as a documented start transient.
    chosen=[]; next_time=t[0]+7_000_000_000
    for i,stamp in enumerate(t):
        if stamp>=next_time:
            x=sample_window(t[:i+1],z[:i+1],int(stamp))
            if x is not None: chosen.append((x,i))
            next_time=int(stamp)+500_000_000
    return chosen


def prepare(root=Path('data')):
    manifest=json.loads((root/'raw/manifest.json').read_text())['episodes']
    groups={k:[] for k in ['train','validation','test']}; seen=set()
    for record in manifest:
        episode=record['episode_id'];assert episode not in seen;seen.add(episode)
        path=root/'raw'/(episode+'.csv')
        assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256']
        for x,i in windows(path):
            groups[record['split']].append((x,CLASSES.index(record['family']),episode,i))
    out=root/'prepared';out.mkdir(parents=True,exist_ok=True)
    summary={}
    for split,items in groups.items():
        np.savez_compressed(out/(split+'.npz'),x=np.stack([r[0] for r in items]),
                            y=np.array([r[1] for r in items]),episodes=np.array([r[2] for r in items]),
                            row_indices=np.array([r[3] for r in items]))
        summary[split]={'episodes':len(set(r[2] for r in items)),'windows':len(items)}
    (out/'manifest.json').write_text(json.dumps({'source':manifest,'features':FEATURES,'history_s':HISTORY_S,
                                               'stride_s':.5,'summary':summary},indent=2))
    return summary
