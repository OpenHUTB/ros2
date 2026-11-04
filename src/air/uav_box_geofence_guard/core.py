import math

def guard(pose, command, age_pose, age_command, horizon=.5, margin=.15):
    if len(pose)!=3 or len(command)!=4: raise ValueError('xyz and vx/vy/vz/yaw required')
    if any(not math.isfinite(v) for v in (*pose,*command,age_pose,age_command)):
        return (0.,0.,0.,0.), 'INVALID'
    if age_pose<0 or age_command<0 or max(age_pose,age_command)>.3:
        return (0.,0.,0.,0.), 'STALE'
    if not math.isfinite(horizon) or horizon<=0 or not math.isfinite(margin) or not 0<=margin<1:
        raise ValueError('invalid horizon or margin')
    x,y,z=pose; vx,vy,vz,yaw=command
    # Fixed teaching box: x/y [-5,5] m, z [0,4] m. ENU world frame only.
    low=(-5+margin,-5+margin,margin); high=(5-margin,5-margin,4-margin)
    if any(not lo<=v<=hi for v,lo,hi in zip(pose,low,high)):
        return (0.,0.,0.,0.), 'OUTSIDE'
    speed=math.sqrt(vx*vx+vy*vy+vz*vz)
    clearance=min(min(v-lo,hi-v) for v,lo,hi in zip(pose,low,high))
    allowed=min(1.5,clearance/horizon)
    scale=min(1.,allowed/speed) if speed else 1.
    return (vx*scale,vy*scale,vz*scale,max(-1.,min(1.,yaw))), ('LIMITED' if scale<1 else 'CLEAR')
