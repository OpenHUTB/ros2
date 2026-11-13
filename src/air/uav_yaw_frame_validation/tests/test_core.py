import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import math,random,unittest
from core import transform,inverse
class Checks(unittest.TestCase):
 def test_identity(self):self.assertEqual(transform((2,3),(0,0),0),(2,3))
 def test_translation(self):self.assertEqual(transform((2,3),(1,-2),0),(3,1))
 def test_quarter_turn(self):
  x,y=transform((1,0),(0,0),math.pi/2);self.assertAlmostEqual(x,0);self.assertAlmostEqual(y,1)
 def test_noncommute(self):self.assertNotEqual(transform((1,0),(2,0),math.pi/2),transform((3,0),(0,0),math.pi/2))
 def test_inverse_random(self):
  rng=random.Random(22)
  for _ in range(500):
   p=(rng.uniform(-100,100),rng.uniform(-100,100));t=(rng.random(),rng.random());y=rng.uniform(-20,20)
   back=inverse(transform(p,t,y),t,y);self.assertLess(math.hypot(back[0]-p[0],back[1]-p[1]),1e-10)
 def test_distance_invariance(self):
  p=transform((2,4),(5,2),1.2);q=transform((3,7),(5,2),1.2);self.assertAlmostEqual(math.dist(p,q),math.sqrt(10))
 def test_nonfinite(self):
  with self.assertRaises(ValueError):transform((1,2),(0,0),float('nan'))
