import copy,json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from uav_radio_link_budget.main import DEFAULT_INPUT,free_space_path_loss_db,estimate_link
class RadioTests(unittest.TestCase):
 def setUp(self):self.p=json.loads(DEFAULT_INPUT.read_text(encoding="utf-8"))
 def test_sample_positive(self):r,e=estimate_link(self.p);self.assertEqual(e,[]);self.assertGreater(r["max_range_m"],0)
 def test_fspl_reference(self):self.assertAlmostEqual(free_space_path_loss_db(2400,1000),100.05,delta=0.1)
 def test_more_tx_power_increases_range(self):a,_=estimate_link(self.p);q=copy.deepcopy(self.p);q["tx_power_dbm"]+=6;b,_=estimate_link(q);self.assertGreater(b["max_range_m"],a["max_range_m"])
 def test_negative_loss_rejected(self):q=copy.deepcopy(self.p);q["additional_loss_db"]=-1;_,e=estimate_link(q);self.assertTrue(e)
 def test_nonpositive_budget_rejected(self):q=copy.deepcopy(self.p);q["tx_power_dbm"]=-100;r,e=estimate_link(q);self.assertEqual(r,{});self.assertTrue(e)
if __name__=="__main__":unittest.main(verbosity=2)
