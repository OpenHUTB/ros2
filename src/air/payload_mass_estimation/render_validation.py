"""Re-run a saved physical flight and verify it before saving simulation evidence."""
import json
from pathlib import Path
import numpy as np
from payload_mass_estimation.simulation import collect
root=Path(__file__).resolve().parent
item=next(a for a in json.loads((root/'data/manifest.json').read_text()) if a['seed']==40120)
(root/'results').mkdir(exist_ok=True)
rows=collect(item['seed'],item['mass_kg'],render_path=str(root/'results/simulation.png'))
with np.load(root/'data'/item['file']) as a:np.testing.assert_array_equal(rows,a['truth'])
print('Rendered physical rerun, identical to recorded flight')
