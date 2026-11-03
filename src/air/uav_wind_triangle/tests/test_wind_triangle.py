import copy, json, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_wind_triangle.main import DEFAULT_INPUT, solve_wind_triangle

class WindTriangleTests(unittest.TestCase):
    def setUp(self): self.profile=json.loads(DEFAULT_INPUT.read_text(encoding="utf-8"))
    def test_sample_solution(self):
        result,errors=solve_wind_triangle(self.profile); self.assertEqual(errors,[]); self.assertGreater(result["ground_speed_ms"],0)
    def test_no_wind_heading_equals_course(self):
        p=copy.deepcopy(self.profile); p["wind_speed_ms"]=0; r,e=solve_wind_triangle(p); self.assertEqual(e,[]); self.assertAlmostEqual(r["required_heading_deg"],p["desired_course_deg"],places=3); self.assertAlmostEqual(r["ground_speed_ms"],p["true_airspeed_ms"],places=3)
    def test_pure_headwind_reduces_ground_speed(self):
        p=copy.deepcopy(self.profile); p["desired_course_deg"]=0; p["wind_from_deg"]=0; r,e=solve_wind_triangle(p); self.assertEqual(e,[]); self.assertAlmostEqual(r["ground_speed_ms"],p["true_airspeed_ms"]-p["wind_speed_ms"],places=3)
    def test_crosswind_requires_heading_correction(self):
        p=copy.deepcopy(self.profile); p["desired_course_deg"]=0; p["wind_from_deg"]=90; r,e=solve_wind_triangle(p); self.assertEqual(e,[]); self.assertNotAlmostEqual(r["wind_correction_deg"],0)
    def test_unreachable_crosswind_is_rejected(self):
        p=copy.deepcopy(self.profile); p["desired_course_deg"]=0; p["wind_from_deg"]=90; p["wind_speed_ms"]=20; r,e=solve_wind_triangle(p); self.assertEqual(r,{}); self.assertTrue(e)

if __name__ == "__main__": unittest.main(verbosity=2)
