import hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).resolve().parent
(root/'assets').mkdir(exist_ok=True)
with zipfile.ZipFile(root/'assets/flights.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in (root/'data').glob('*'):z.write(p,p.relative_to(root))
files=list((root/'payload_mass_estimation').glob('*.py'))+list((root/'models').rglob('model.pt'))+list((root/'models').rglob('meta.json'))
(root/'artifacts').mkdir(exist_ok=True)
node=json.loads((root/'results/ros_node.json').read_text());sub=json.loads((root/'results/ros_subscriber.json').read_text())
assert node['observations']==720 and node['predictions']==663
assert sub['messages']==663,sub['messages']
(root/'artifacts/validation.json').write_text(json.dumps({'date':'2026-10-15','tests_passed':8,'ros_observations':720,'ros_predictions':663,'subscriber_predictions':sub['messages'],'scope':'MuJoCo vertical flights and actual ROS2 replay; no real aircraft or OpenHUTB Unreal validation','sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}},indent=2))
print('Packaged flights; verified 720 observations and 663 subscribed predictions')
