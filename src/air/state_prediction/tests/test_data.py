import unittest
import numpy as np
from prediction.data import features,baseline,HORIZONS


class DataSemantics(unittest.TestCase):
    def test_translation_invariance(self):
        x=np.zeros((2,20,16),dtype=np.float32)
        x[:,:,0]=np.arange(20)
        moved=x.copy(); moved[:,:,:3]+=np.array([100,-40,2])
        np.testing.assert_allclose(features(x),features(moved))
        np.testing.assert_array_equal(features(x)[:,-1,:3],0)

    def test_physical_baselines(self):
        state=np.zeros((1,16),dtype=np.float32); state[0,:9]=[1,2,3,2,-1,0,1,0,-2]
        pred=baseline(state,True)
        for i,h in enumerate(HORIZONS):
            np.testing.assert_allclose(pred[0,i,:3],state[0,:3]+state[0,3:6]*h+0.5*state[0,6:9]*h*h)
            np.testing.assert_allclose(pred[0,i,3:],state[0,3:6]+state[0,6:9]*h)


if __name__=='__main__': unittest.main()
