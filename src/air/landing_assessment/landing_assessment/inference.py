"""Online estimator sees only causal odometry; no contact or mission labels."""
from collections import deque
import numpy as np
from .data import WINDOW,PHASES
from .simulation import angles,FOOT_Z
from .learning import load,predict

class Estimator:
    def __init__(self,model):
        self.net,self.meta=load(model);self.reset()
    def reset(self):
        self.buffer=deque(maxlen=WINDOW);self.last_time=None;self.last_velocity=None
        self.last_height=None;self.impact_issued=False
    def step(self,t,position,velocity,quaternion,omega):
        values=np.r_[t,position,velocity,quaternion,omega]
        if not np.isfinite(values).all() or abs(np.linalg.norm(quaternion)-1)>.05:
            self.reset();return {'ready':False,'reason':'invalid_odometry'}
        velocity=np.asarray(velocity)
        if self.last_time is not None and not .01<=t-self.last_time<=.15:
            self.reset()
        acceleration=np.zeros(3) if self.last_time is None else (velocity-self.last_velocity)/(t-self.last_time)
        height=float(position[2]-FOOT_Z)
        feature=np.r_[height,velocity,acceleration,angles(quaternion)[:2],omega].astype(np.float32)
        self.buffer.append(feature);self.last_velocity=velocity.copy();self.last_time=t
        crossing=self.last_height is not None and self.last_height>.18 and height<=.18
        self.last_height=height
        result={'time_s':float(t),'height_m':height,'ready':len(self.buffer)==WINDOW,'reason':'warming_up'}
        if len(self.buffer)==WINDOW:
            prob,impact=predict(self.net,self.meta,np.array(self.buffer)[None])
            index=int(prob[0].argmax());result.update(reason='ok',phase=PHASES[index],phase_index=index,confidence=float(prob[0,index]))
            if crossing and not self.impact_issued:
                result['predicted_peak_load_factor']=float(impact[0]);self.impact_issued=True
        return result
