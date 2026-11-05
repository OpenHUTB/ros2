import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json,unittest
from pathlib import Path
import numpy as np
from core import dataset,predict,WINDOW
class Checks(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.model=json.loads((Path(__file__).resolve().parents[1]/'model.json').read_text())
 def test_windows(self):
  x,y,_,_=dataset(1,sequences=2,length=20);self.assertEqual(x.shape,(28,WINDOW));self.assertEqual(y.shape,(28,1))
 def test_determinism(self):self.assertTrue(np.array_equal(dataset(2)[0],dataset(2)[0]))
 def test_separate_sequences(self):self.assertFalse(np.array_equal(dataset(101)[0],dataset(909)[0]))
 def test_finite_prediction(self):self.assertTrue(np.isfinite(predict(dataset(909,sequences=1)[0],self.model)).all())
 def test_serialization(self):
  x=dataset(88,sequences=1)[0];self.assertTrue(np.array_equal(predict(x,self.model),predict(x,json.loads(json.dumps(self.model)))))
 def test_invalid_input(self):
  with self.assertRaises(ValueError):predict([[float('nan')]*7],self.model)
 def test_invalid_shape(self):
  with self.assertRaises(ValueError):predict([[1,2]],self.model)
 def test_heldout_improves_raw(self):
  x,y,raw,_=dataset(808,sequences=3);truth=y.ravel()*4
  self.assertLess(np.mean((predict(x,self.model).ravel()-truth)**2),np.mean((raw-truth)**2))
