"""Causal windows. Contact, phase, force, and commands never enter features."""
import hashlib
import json
from pathlib import Path
import numpy as np
from .simulation import angles,FOOT_Z

WINDOW=8
FEATURES=['height_m','vx','vy','vz','ax','ay','az','roll','pitch','wx','wy','wz']
PHASES=['approach','descent','touchdown','settled']


def observable(raw):
    velocities=raw[:,4:7]
    acceleration=np.zeros_like(velocities)
    acceleration[1:]=np.diff(velocities,axis=0)/np.diff(raw[:,0])[:,None]
    attitude=np.array([angles(q) for q in raw[:,7:11]])[:,:2]
    return np.c_[raw[:,3]-FOOT_Z,velocities,acceleration,attitude,raw[:,11:14]].astype(np.float32)


def prepare(folder):
    folder=Path(folder);manifest=json.loads((folder/'manifest.json').read_text())
    splits={}
    for record in manifest:
        path=folder/record['file']
        assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256'],'Corpus checksum mismatch'
        raw=np.load(path)['raw'];features=observable(raw)
        assert np.isfinite(raw).all() and np.all(np.diff(raw[:,0])>0)
        # Decision is the first observed crossing below 18 cm, strictly before contact.
        crossings=np.flatnonzero((features[:,0]<=.18)&(raw[:,0]<record['contact_time_s']))
        if not len(crossings):raise ValueError('No causal pre-contact observation')
        gate=int(crossings[0]);assert gate>=WINDOW-1
        out=splits.setdefault(record['split'],{'x':[],'y':[],'episode':[],'qx':[],'qy':[],'qepisode':[]})
        for j in range(WINDOW-1,len(raw),2):
            out['x'].append(features[j-WINDOW+1:j+1]);out['y'].append(int(raw[j,15]));out['episode'].append(record['seed'])
        out['qx'].append(features[gate-WINDOW+1:gate+1]);out['qy'].append(record['peak_load_factor']);out['qepisode'].append(record['seed'])
    for out in splits.values():
        for key in out:out[key]=np.asarray(out[key])
    ids=[set(v['qepisode'].tolist()) for v in splits.values()]
    assert all(not a.intersection(b) for i,a in enumerate(ids) for b in ids[i+1:])
    return splits
