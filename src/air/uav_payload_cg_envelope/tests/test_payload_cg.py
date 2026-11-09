import copy,json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from uav_payload_cg_envelope.main import DEFAULT_INPUT,check_cg
class CgTests(unittest.TestCase):
 def setUp(self):self.p=json.loads(DEFAULT_INPUT.read_text(encoding="utf-8"))
 def test_sample_pass(self):r,e=check_cg(self.p);self.assertEqual(e,[]);self.assertTrue(r["feasible"])
 def test_forward_payload_shifts_cg_forward(self):a,e=check_cg(self.p);q=copy.deepcopy(self.p);q["payloads"][1]["cg_m"][0]=0.5;b,e=check_cg(q);self.assertGreater(b["cg_x_m"],a["cg_x_m"])
 def test_outside_longitudinal_limit_fails(self):q=copy.deepcopy(self.p);q["payloads"][1]["cg_m"][0]=10;r,e=check_cg(q);self.assertEqual(e,[]);self.assertFalse(r["feasible"])
 def test_excess_lateral_offset_fails(self):q=copy.deepcopy(self.p);q["payloads"][0]["cg_m"][1]=1;r,e=check_cg(q);self.assertEqual(e,[]);self.assertFalse(r["feasible"])
 def test_zero_payload_mass_rejected(self):q=copy.deepcopy(self.p);q["payloads"][0]["mass_kg"]=0;r,e=check_cg(q);self.assertTrue(e)
 def test_overlarge_mass_is_rejected(self):q=copy.deepcopy(self.p);q["empty"]["mass_kg"]=1e308;q["payloads"][0]["mass_kg"]=1e308;r,e=check_cg(q);self.assertEqual(r,{});self.assertTrue(any("must not exceed" in x for x in e))
 def test_nonfinite_combined_moment_is_rejected(self):q=copy.deepcopy(self.p);q["empty"]["cg_m"][0]=1e308;r,e=check_cg(q);self.assertEqual(r,{});self.assertTrue(any("combined mass and moments must be finite" in x for x in e))
if __name__=="__main__":unittest.main(verbosity=2)
