"""Summarize saved final metrics without changing any model or threshold."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    out=Path('results/final')
    with (out/'by_type.csv').open() as f: rows=list(csv.DictReader(f))
    details=json.loads((out/'details.json').read_text())
    summary=[]
    for name in sorted({r['method'] for r in rows}):
        group=[r for r in rows if r['method']==name]
        tp=sum(int(r['detected']) for r in group);total=sum(int(r['events']) for r in group)
        faults=[d for d in details if d['method']==name and d['kind']!='clean']
        fp=sum(d['false_alarm_events'] for d in faults)
        precision=tp/max(tp+fp,1);recall=tp/total
        summary.append({'method':name,'events':total,'detected':tp,'precision':precision,'recall':recall,
                        'f1':2*precision*recall/max(precision+recall,1e-12),
                        'clean_false_alarms_per_min':float(group[0]['clean_false_alarms_per_min'])})
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    selected=[s for s in summary if s['method'] in ['consistency_42','consistency_mlp_42','trapezoid_rules','timestamp_rules','no_persistence']]
    fig,ax=plt.subplots(1,2,figsize=(12,4))
    names=[s['method'] for s in selected]
    ax[0].bar(names,[s['recall'] for s in selected]);ax[0].set(ylabel='Recall',ylim=(0,1.1))
    ax[1].bar(names,[s['clean_false_alarms_per_min'] for s in selected]);ax[1].set(ylabel='False alarms / clean flight minute')
    for a in ax:a.tick_params(axis='x',rotation=30);a.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(out/'ablation.png',dpi=150);plt.close(fig)
    seeds=[s for s in summary if s['method'] in ['consistency_42','consistency_43','consistency_44']]
    seed_summary={key:{'mean':float(np.mean([s[key] for s in seeds])),
                       'population_std':float(np.std([s[key] for s in seeds]))}
                  for key in ['recall','f1','clean_false_alarms_per_min']}
    (out/'seed_summary.json').write_text(json.dumps(seed_summary,indent=2))
    print(json.dumps(summary))


if __name__=='__main__':main()
