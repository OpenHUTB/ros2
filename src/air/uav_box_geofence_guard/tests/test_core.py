import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import math,unittest
from core import guard
class Checks(unittest.TestCase):
 def test_center(self):self.assertEqual(guard((0,0,2),(1,0,0,.2),0,0),((1.,0.,0.,.2),'CLEAR'))
 def test_boundary_stop(self):self.assertEqual(guard((4.85,0,2),(1,0,0,0),0,0)[0][0],0)
 def test_diagonal_ball(self):
  values,state=guard((4.7,0,2),(1,1,0,0),0,0)
  self.assertLessEqual(math.hypot(*values[:2])*.5,.15+1e-12)
 def test_stale(self):self.assertEqual(guard((0,0,2),(1,0,0,0),.31,0)[1],'STALE')
 def test_nonfinite(self):self.assertEqual(guard((0,0,2),(float('nan'),0,0,0),0,0)[1],'INVALID')
 def test_outside(self):self.assertEqual(guard((0,0,5),(0,0,-1,0),0,0)[1],'OUTSIDE')
 def test_horizon(self):
  with self.assertRaises(ValueError):guard((0,0,2),(1,0,0,0),0,0,horizon=0)
 def test_swept_bound(self):
  import random
  rng=random.Random(72)
  for _ in range(500):
   p=tuple(rng.uniform(a,b) for a,b in [(-4.85,4.85),(-4.85,4.85),(.15,3.85)])
   c=tuple(rng.uniform(-3,3) for _ in range(4));v,_=guard(p,c,0,0)
   for axis in range(3):
    self.assertGreaterEqual(p[axis]+v[axis]*.5,(-4.85,-4.85,.15)[axis]-1e-12)
    self.assertLessEqual(p[axis]+v[axis]*.5,(4.85,4.85,3.85)[axis]+1e-12)
