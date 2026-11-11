import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from airsim_rl_planner_ros1.depth_guard import DepthGuard


class GuardTests(unittest.TestCase):
    def test_clear_speed_limit(self):
        v,_=DepthGuard().filter([1,0,0],np.full((24,30),10.),[0,0,0,1],0)
        np.testing.assert_allclose(v,[.5,0,0])

    def test_solid_wall_and_bad_depth_stop(self):
        for d in (np.full((24,30),2.),np.full((24,30),np.nan)):
            v,_=DepthGuard().filter([1,0,0],d,[0,0,0,1],0)
            np.testing.assert_array_equal(v,[0,0,0])

    def test_bypass_expiry_and_hysteresis(self):
        g=DepthGuard(); d=np.full((24,30),2.5);d[:,:10]=8.
        v,_=g.filter([1,0,0],d,[0,0,0,1],0)
        np.testing.assert_allclose(v,[0,-.2,0])
        d[:,10:20]=3.5
        v,_=g.filter([1,0,0],d,[0,0,0,1],1)
        self.assertLess(v[1],0)
        v,_=g.filter([1,0,0],d,[0,0,0,1],2.1)
        np.testing.assert_array_equal(v,[0,0,0])

    def test_rotated_camera(self):
        d=np.full((24,30),2.5);d[:,:10]=8.
        v,_=DepthGuard().filter([0,1,0],d,[0,0,2**-.5,2**-.5],0)
        np.testing.assert_allclose(v,[.2,0,0],atol=1e-8)

    def test_unobserved_motion_stop(self):
        for action in ([0,0,1],[-1,0,0],[0,1,0]):
            v,_=DepthGuard().filter(action,np.full((24,30),10.),[0,0,0,1],0)
            np.testing.assert_array_equal(v,[0,0,0])
