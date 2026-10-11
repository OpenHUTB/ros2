import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
import torch
from pattern_model import PatternNet
from pattern_data import CLASSES


def metrics(truth,pred):
    cm=np.zeros((5,5),int)
    for t,p in zip(truth,pred):cm[int(t),int(p)]+=1
    precision=np.divide(np.diag(cm),cm.sum(0),out=np.zeros(5),where=cm.sum(0)>0)
    recall=np.divide(np.diag(cm),cm.sum(1),out=np.zeros(5),where=cm.sum(1)>0)
    f1=np.divide(2*precision*recall,precision+recall,out=np.zeros(5),where=precision+recall>0)
    return {'accuracy':float((truth==pred).mean()),'macro_f1':float(f1.mean()),'confusion_matrix':cm.tolist(),
            'per_class':[{ 'class':name,'precision':float(p),'recall':float(r),'f1':float(f)} for name,p,r,f in zip(CLASSES,precision,recall,f1)]}


def evaluate(split='test',output=Path('results'),models=Path('models')):
    torch.set_num_threads(3);output.mkdir(parents=True,exist_ok=True)
    data=np.load('data/prepared/'+split+'.npz');train=np.load('data/prepared/train.npz')
    assert not set(data['episodes'])&set(train['episodes'])
    summaries=[]
    for name in ['mlp_42','mlp_43','mlp_44','gru_42','mlp_position_only_42']:
        folder=models/name;c=json.loads((folder/'config.json').read_text())
        model=PatternNet(c['variant']);model.load_state_dict(torch.load(folder/'model.pt',weights_only=True));model.eval()
        x=torch.from_numpy(((data['x']-c['mean'])/c['std']).astype(np.float32))
        with torch.no_grad():p=torch.softmax(model(x),1).numpy()
        pred=p.argmax(1);conf=p.max(1);accepted=conf>=c['confidence_threshold']
        result=metrics(data['y'],pred);result.update(model=name,windows=len(x),episodes=len(set(data['episodes'])),
            accepted_coverage=float(accepted.mean()),accepted_accuracy=float((pred[accepted]==data['y'][accepted]).mean()) if accepted.any() else None)
        episodes=[]
        for ep in sorted(set(data['episodes'])):
            mask=data['episodes']==ep;estimate=int(np.argmax(np.bincount(pred[mask],minlength=5)));truth=int(data['y'][mask][0])
            episodes.append({'episode':str(ep),'truth':CLASSES[truth],'majority_prediction':CLASSES[estimate],'correct':estimate==truth})
        result['episode_results']=episodes;result['episode_accuracy']=sum(e['correct'] for e in episodes)/len(episodes)
        with torch.no_grad():
            model(x[:1]);durations=[]
            for _ in range(100):
                start=time.perf_counter();model(x[:1]);durations.append((time.perf_counter()-start)*1000)
        result['inference_median_ms']=float(np.median(durations));result['inference_p95_ms']=float(np.quantile(durations,.95))
        (output/(name+'.json')).write_text(json.dumps(result,indent=2))
        summaries.append({k:v for k,v in result.items() if k not in ['confusion_matrix','per_class','episode_results']})
        with (output/(name+'_predictions.csv')).open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['episode','row_index','truth','prediction','confidence','accepted'])
            w.writerows(zip(data['episodes'],data['row_indices'],[CLASSES[i] for i in data['y']],[CLASSES[i] for i in pred],conf,accepted))
    # Fixed, non-neural baseline on temporal feature means and standard deviations.
    def stats(x):return np.c_[x.mean(1),x.std(1)]
    tr,te=stats(train['x']),stats(data['x']);mean=tr.mean(0);std=np.maximum(tr.std(0),.02)
    tr=(tr-mean)/std;te=(te-mean)/std
    centers=np.stack([tr[train['y']==i].mean(0) for i in range(5)])
    pred=((te[:,None]-centers[None])**2).sum(2).argmin(1)
    baseline=metrics(data['y'],pred);baseline['model']='nearest_centroid'
    (output/'nearest_centroid.json').write_text(json.dumps(baseline,indent=2))
    summaries.append({k:v for k,v in baseline.items() if k not in ['confusion_matrix','per_class']})
    (output/'summary.json').write_text(json.dumps(summaries,indent=2));print(json.dumps(summaries),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--split',default='test');p.add_argument('--output',type=Path,default=Path('results'))
    p.add_argument('--models',type=Path,default=Path('models'))
    a=p.parse_args();evaluate(a.split,a.output,a.models)
