"""Online estimator sees only causal odometry; no contact or mission labels."""
from collections import deque
import numpy as np
from .data import WINDOW,PHASES
from .simulation import angles,FOOT_Z
from .learning import load,predict,interval

# 风险分级阈值来自训练集峰值载荷的 P75/P90；也可以显式传入覆盖。
def risk_level(high,elevated=None,high_limit=None):
    """按区间上界给出保守风险等级：low / elevated / high。"""
    if high_limit is not None and high>high_limit:return 'high'
    if elevated is not None and high>elevated:return 'elevated'
    return 'low'


class Estimator:
    def __init__(self,model,risk_thresholds=None):
        self.net,self.meta=load(model)
        # 默认用训练集分位数；显式传入 {elevated, high} 可覆盖（例如按机体结构设定）
        self.risk_thresholds=risk_thresholds or self.meta.get('risk_thresholds') or {}
        self.reset()
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
                point=float(impact[0]);low,high=interval(self.meta,point)
                result['predicted_peak_load_factor']=point
                result['predicted_peak_load_factor_low']=float(low)
                result['predicted_peak_load_factor_high']=float(high)
                result['risk_level']=risk_level(high,self.risk_thresholds.get('elevated'),
                                                self.risk_thresholds.get('high'))
                self.impact_issued=True
        return result
