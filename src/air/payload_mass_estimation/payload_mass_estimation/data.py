"""Causal windows shared by offline evaluation and ROS inference."""
import numpy as np
WINDOW=20
EMPTY_MASS=.95
DT=.05

def features(rows):
    a=np.asarray(rows,dtype=float)
    # row t, height, measured velocity, commanded thrust; thrust at k-1 caused dv[k].
    acceleration=np.diff(a[:,2])/DT
    force=a[:-1,3]
    specific=acceleration+9.81
    mass=force.sum()/max(specific.sum(),1e-9)
    return np.r_[force,acceleration], float(mass-EMPTY_MASS)

class Estimator:
    def __init__(self, predict):self.predict=predict;self.rows=[]
    def step(self,row):
        row=np.asarray(row,float)
        if row.shape!=(4,) or not np.isfinite(row).all():self.rows=[];return None
        if self.rows and abs(row[0]-self.rows[-1][0]-DT)>DT*.25:self.rows=[]
        self.rows.append(row);self.rows=self.rows[-WINDOW:]
        if len(self.rows)<WINDOW:return None
        x,baseline=features(self.rows)
        return {'mass_kg':float(self.predict(x)), 'physics_kg':baseline}
