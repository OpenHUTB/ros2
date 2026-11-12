import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
from core import Alarm
class Checks(unittest.TestCase):
 def test_confirm(self):
  a=Alarm();self.assertNotEqual(a.update(.8,0),'ALARM');a.update(.8,.1);self.assertEqual(a.update(.8,.2),'ALARM')
 def test_hysteresis(self):
  a=Alarm(confirm=1);a.update(.8,0);self.assertEqual(a.update(1.1,.1),'ALARM');self.assertEqual(a.update(1.4,.2),'CLEAR')
 def test_broken_streak(self):
  a=Alarm();a.update(.8,0);a.update(1.1,.1);a.update(.8,.2);self.assertNotEqual(a.update(.8,.3),'ALARM')
 def test_gap_not_continuous(self):
  a=Alarm();a.update(.8,0);a.update(.8,.1);self.assertNotEqual(a.update(.8,1),'ALARM')
 def test_stale(self):
  a=Alarm();a.update(2,0);self.assertEqual(a.check(.6),'STALE')
 def test_invalid(self):
  for v in [float('inf'),float('nan'),-1]:self.assertEqual(Alarm().update(v,0),'INVALID')
 def test_clock_backward(self):
  a=Alarm();a.update(2,1);self.assertEqual(a.check(.5),'INVALID')
 def test_parameters(self):
  with self.assertRaises(ValueError):Alarm(low=2,high=1)
