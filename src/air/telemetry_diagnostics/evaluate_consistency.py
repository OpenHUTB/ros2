"""Final, newly acquired flight evaluation. No fitting or threshold selection here."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from consistency import ConsistencyDetector, extract, transform
from holdout_evaluate import scenarios, score_events
from telemetry_data import write_csv
from train_evaluate import sustained


def main():
    torch.set_num_threads(3)
    out = Path('results/final'); out.mkdir(parents=True, exist_ok=True)
    folders = [Path('models') / n for n in ['consistency_42','consistency_43','consistency_44','consistency_mlp_42']]
    detectors = {p.name: ConsistencyDetector(p) for p in folders}
    protocol = json.loads(Path('final_protocol.json').read_text())
    assert protocol['deployment_model'] == 'consistency_42'
    for path, digest in protocol['model_hashes'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
    files = sorted(Path('data/final_raw').glob('*.csv'))
    assert len(files) == 10
    development = set(r['episode'] for r in json.loads((folders[0]/'development_split.json').read_text()))
    assert not development.intersection(p.stem for p in files)
    details = []; manifest = []
    for raw in files:
        meta = json.loads(raw.with_suffix('.json').read_text())
        assert hashlib.sha256(raw.read_bytes()).hexdigest() == meta['sha256']
        assert meta['collision_count'] == 0 and meta['timestamps_strictly_increasing']
        with raw.open() as f: rows = list(csv.DictReader(f))
        for kind,severity,observations,labels in scenarios(rows):
            axes = range(3) if kind in ('smooth_position_drift','velocity_bias') else [0]
            for axis in axes:
                obs = [r.copy() for r in observations]
                if axis:
                    prefix = 'p' if kind == 'smooth_position_drift' else 'v'
                    for r, original in zip(obs,rows):
                        delta=float(r[prefix+'x'])-float(original[prefix+'x'])
                        r[prefix+'x']=float(original[prefix+'x'])
                        r[prefix+'xyz'[axis]]=float(original[prefix+'xyz'[axis]])+delta
                x,idx,rules=extract(obs); n=len(obs)
                t=np.array([(int(r['arrival_ns'])-int(obs[0]['arrival_ns']))/1e9 for r in obs])
                methods={}
                for name,d in detectors.items():
                    c=d.config
                    with torch.no_grad(): logits=d.model(torch.from_numpy(transform(x,np.array(c['scale'])))).numpy()
                    flags=np.zeros(n,bool); flags[idx]=logits>c['threshold']
                    methods[name]=sustained(flags|rules,3)
                    if name=='consistency_42':
                        methods['neural_only']=sustained(flags,3)
                        methods['no_persistence']=flags|rules
                        pflags=np.zeros(n,bool)
                        pflags[idx]=np.sqrt(np.mean(transform(x,np.array(c['scale']))[:,-1]**2,axis=1))>c['physics_threshold']
                        methods['trapezoid_rules']=sustained(pflags|rules,3)
                methods['timestamp_rules']=sustained(rules,3)
                for name,alarm in methods.items():
                    details.append(dict(episode=raw.stem,family=meta['family'],axis='xyz'[axis],kind=kind,
                                        severity=severity,method=name,duration_s=float(t[-1]),
                                        **score_events(alarm,labels,t)))
                if raw==files[0] and axis==0:
                    folder=Path('data/final_demo')/kind/str(severity);folder.mkdir(parents=True,exist_ok=True)
                    write_csv(folder/'observations.csv',obs)
                    write_csv(folder/'evaluation_only.csv',[{'arrival_ns':r['arrival_ns'],'anomaly':int(l)} for r,l in zip(obs,labels)])
                manifest.append({'episode':raw.stem,'kind':kind,'severity':severity,'axis':'xyz'[axis],
                                 'sha256':meta['sha256']})
    summary=[]
    for name in sorted({d['method'] for d in details}):
        clean=[d for d in details if d['method']==name and d['kind']=='clean']
        fpmin=sum(d['false_alarm_events'] for d in clean)/(sum(d['duration_s'] for d in clean)/60)
        for kind,severity in sorted({(d['kind'],d['severity']) for d in details if d['kind']!='clean'}):
            group=[d for d in details if d['method']==name and d['kind']==kind and d['severity']==severity]
            tp=sum(d['detected'] for d in group);fp=sum(d['false_alarm_events'] for d in group)
            precision=tp/max(tp+fp,1);recall=tp/len(group)
            delays=[d['delay_s'] for d in group if d['delay_s'] is not None]
            summary.append({'method':name,'kind':kind,'severity':severity,'events':len(group),
                            'detected':tp,'missed':len(group)-tp,'precision':precision,'recall':recall,
                            'f1':2*precision*recall/max(precision+recall,1e-12),
                            'mean_detected_delay_s':float(np.mean(delays)) if delays else None,
                            'clean_false_alarms_per_min':fpmin})
    write_csv(out/'by_type.csv',summary)
    (out/'details.json').write_text(json.dumps(details,indent=2))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps([s for s in summary if s['method']=='consistency_42']),flush=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for ax,kind in zip(axes,('smooth_position_drift','velocity_bias')):
        for name in ('consistency_42','consistency_mlp_42','trapezoid_rules'):
            group=[s for s in summary if s['method']==name and s['kind']==kind]
            ax.plot([s['severity'] for s in group],[s['recall'] for s in group],'o-',label=name)
        ax.set(title=kind,xlabel='Severity (m/s)',ylabel='Event recall',ylim=(-.05,1.05));ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'severity_recall.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4))
    for folder in folders:
        log=json.loads((folder/'loss.json').read_text())
        ax.plot([d['epoch'] for d in log],[d['validation_loss'] for d in log],label=folder.name)
    ax.set(xlabel='Epoch',ylabel='Validation weighted BCE');ax.legend();ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'loss.png',dpi=150)


if __name__=='__main__':main()
