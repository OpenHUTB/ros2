import numpy as np

WINDOW=7

def dataset(seed, sequences=20, length=160):
    rng=np.random.RandomState(seed); xs=[]; ys=[]; noisy_values=[]; means=[]
    for _ in range(sequences):
        t=np.arange(length)*.05
        phase=rng.uniform(-np.pi,np.pi)
        height=rng.uniform(1.5,2.5)+rng.uniform(.15,.6)*np.sin(rng.uniform(.3,1.2)*t+phase)
        noisy=height+rng.normal(0,.15,length)
        for i in range(WINDOW-1,length):
            xs.append(noisy[i-WINDOW+1:i+1]/4.)
            ys.append(height[i]/4.); noisy_values.append(noisy[i]); means.append(np.mean(noisy[i-WINDOW+1:i+1]))
    return np.asarray(xs),np.asarray(ys)[:,None],np.asarray(noisy_values),np.asarray(means)

def train(x,y,steps=800,seed=42):
    rng=np.random.RandomState(seed)
    parameters=[rng.normal(0,.25,(WINDOW,16)),np.zeros((1,16)),rng.normal(0,.1,(16,1)),np.zeros((1,1))]
    first=[np.zeros_like(v) for v in parameters]; second=[np.zeros_like(v) for v in parameters]
    for step in range(1,steps+1):
        a,b,c,d=parameters; hidden=np.tanh(x@a+b); prediction=hidden@c+d
        grad=2*(prediction-y)/len(x); dh=(grad@c.T)*(1-hidden*hidden)
        gradients=[x.T@dh,dh.sum(axis=0,keepdims=True),hidden.T@grad,grad.sum(axis=0,keepdims=True)]
        for i,g in enumerate(gradients):
            first[i]=.9*first[i]+.1*g; second[i]=.999*second[i]+.001*g*g
            parameters[i]-=.01*(first[i]/(1-.9**step))/(np.sqrt(second[i]/(1-.999**step))+1e-8)
    return parameters

def predict(x,model):
    x=np.asarray(x,dtype=float)
    if x.ndim!=2 or x.shape[1]!=WINDOW or not np.isfinite(x).all():
        raise ValueError('finite Nx7 normalized input required')
    a,b,c,d=[np.asarray(v,dtype=float) for v in model]
    if [v.shape for v in (a,b,c,d)]!=[(7,16),(1,16),(16,1),(1,1)]:
        raise ValueError('invalid model shapes')
    if not all(np.isfinite(v).all() for v in (a,b,c,d)): raise ValueError('invalid model values')
    return (np.tanh(x@a+b)@c+d)*4.
