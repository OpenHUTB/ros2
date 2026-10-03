import unittest
import numpy as np
from prediction.ros_nodes import enu,enu_flu_orientation,quat_product


class Frames(unittest.TestCase):
    def test_north_east_down_to_east_north_up(self):
        self.assertEqual(enu([2,3,-4]),(3.,2.,4.))

    def test_identity_ned_frd_heading(self):
        q=enu_flu_orientation([1,0,0,0])
        # Body forward maps to north (+ENU y), body left to west (-ENU x).
        conjugate=q*np.array([1,-1,-1,-1])
        forward=quat_product(quat_product(q,[0,1,0,0]),conjugate)[1:]
        left=quat_product(quat_product(q,[0,0,1,0]),conjugate)[1:]
        np.testing.assert_allclose(forward,[0,1,0],atol=1e-6)
        np.testing.assert_allclose(left,[-1,0,0],atol=1e-6)


if __name__=='__main__': unittest.main()
