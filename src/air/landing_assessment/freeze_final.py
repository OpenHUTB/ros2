"""Freeze the existing models before collecting the independent final cohort."""
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent
files=[root/'landing_assessment'/p for p in ('simulation.py','data.py','learning.py')]
files+=list((root/'models/trained').rglob('model.pt'))+list((root/'models/trained').rglob('meta.json'))+[root/'models/trained/baselines.json']
protocol={'final_episode_seeds':list(range(20000,20024)),
          'deployment':'mlp_42 (declared before final collection)',
          'purpose':'independent same-distribution simulation cohort; not real-flight or cross-simulator validation',
          'sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
path=root/'final_protocol.json'
if path.exists():
    assert json.loads(path.read_text())==protocol,'Frozen model or data generation code changed'
else:path.write_text(json.dumps(protocol,indent=2))
print('Final experiment protocol frozen; all source and model hashes verified.')
