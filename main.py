"""Tabular Q-learning versus validation-tuned PID on a simulated second-order plant."""
from pathlib import Path
import json, argparse, platform
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DT=.05; N=200; ACTIONS=np.linspace(-3,3,9)
EDGES=np.linspace(-2,2,32)

def state(x,v): return int(np.digitize(1-x,EDGES)),int(np.digitize(v,EDGES))
def step(x,v,u,a,b,d):
    v=v+DT*(-a*v-b*x+u+d)
    return x+DT*v,v

def scenario(seed, variant=False):
    rng=np.random.default_rng(seed)
    if variant: a,b=rng.uniform(.4,1.2),rng.uniform(.6,1.4)
    else: a,b=1.,1.
    return float(rng.uniform(-.2,.2)),float(rng.uniform(-.1,.1)),a,b

def rollout(kind, seed, q=None, gains=None, variant=False):
    x,v,a,b=scenario(seed,variant); integ=0; rows=[]
    for t in range(N):
        error=1-x
        if kind=='PID':
            kp,ki,kd=gains
            raw=kp*error+ki*integ-kd*v
            u=float(np.clip(raw,-3,3))
            # Conditional integration: avoid winding up further into saturation.
            if abs(raw)<3 or raw*error<0: integ+=error*DT
        else: u=float(ACTIONS[np.argmax(q[state(x,v)])])
        d=.4 if t>=N//2 else 0.
        rows.append((t*DT,x,v,u,error))
        x,v=step(x,v,u,a,b,d)
    z=np.array(rows); e=z[:,4]
    outside=np.flatnonzero(abs(e)>.02)
    settling=(outside[-1]+1)*DT if len(outside) and outside[-1]<N-1 else (0. if not len(outside) else None)
    return z, {'iae':float(np.sum(abs(e))*DT),'rmse':float(np.sqrt(np.mean(e**2))),
      'overshoot_percent':float(max(0,z[:,1].max()-1)*100),'final_absolute_error':float(abs(e[-1])),
      'control_effort':float(np.sum(z[:,3]**2)*DT),'settling_time_seconds':settling}

def train(seed, episodes):
    rng=np.random.default_rng(seed); q=np.zeros((33,33,len(ACTIONS)))
    returns=[]
    for ep in range(episodes):
        x,v,a,b=scenario(int(rng.integers(1,10000000)),True); total=0
        epsilon=max(.03,.8*(1-ep/episodes))
        for t in range(N):
            s=state(x,v)
            action=int(rng.integers(len(ACTIONS))) if rng.random()<epsilon else int(np.argmax(q[s]))
            u=ACTIONS[action]; d=float(rng.uniform(-.5,.5)) if t>=N//2 else 0
            nx,nv=step(x,v,u,a,b,d)
            reward=-((1-nx)**2+.03*u*u+.05*nv*nv)
            target=reward if t==N-1 else reward+.98*np.max(q[state(nx,nv)])
            q[s+(action,)]+=.12*(target-q[s+(action,)])
            x,v=nx,nv; total+=reward
        returns.append(total)
    return q,returns

def main():
    p=argparse.ArgumentParser(); p.add_argument('--episodes',type=int,default=2000); args=p.parse_args()
    out=Path('results'); out.mkdir(exist_ok=True)
    # Tune PID on validation seeds; final test seeds below are disjoint.
    candidates=[(kp,ki,kd) for kp in [2.,4.,6.] for ki in [.5,1.,2.] for kd in [1.,2.,3.]]
    gains=min(candidates,key=lambda g:np.mean([rollout('PID',s,gains=g,variant=True)[1]['iae'] for s in range(10,20)]))
    rows=[]; curves=[]; sample=None
    for train_seed in [0,1,2]:
        q,ret=train(train_seed,args.episodes); curves.append(ret)
        np.save(out/f'q_table_seed_{train_seed}.npy',q)
        for variant in [False,True]:
            for seed in range(100,120):
                for method in ['PID','Q-learning']:
                    z,m=rollout(method,seed,q=q,gains=gains,variant=variant)
                    rows.append(dict(training_seed=train_seed,test_seed=seed,scenario='parameter_variation' if variant else 'nominal',controller=method,**m))
                    if train_seed==0 and seed==100 and not variant:
                        if sample is None: sample={}
                        sample[method]=z
    df=pd.DataFrame(rows); df.to_csv(out/'all_trials.csv',index=False)
    # PID is deterministic and repeated for pairing; remove training-seed duplication for its summary.
    pid=df[df.controller=='PID'].drop_duplicates(['test_seed','scenario','controller'])
    summary=pd.concat([pid,df[df.controller!='PID']]).groupby(['scenario','controller'])[['iae','rmse','overshoot_percent','final_absolute_error','control_effort','settling_time_seconds']].agg(['mean','std'])
    summary.to_csv(out/'summary.csv')
    clean=pd.concat([pid,df[df.controller!='PID']]).copy()
    clean['settled']=clean.settling_time_seconds.notna()
    clean.groupby(['scenario','controller']).settled.mean().to_csv(out/'settling_success_rate.csv')
    pd.DataFrame(np.array(curves).T,columns=['seed_0','seed_1','seed_2']).to_csv(out/'training_returns.csv',index=False)
    fig,ax=plt.subplots(figsize=(8,4))
    for method,z in sample.items(): ax.plot(z[:,0],z[:,1],label=method)
    ax.axhline(1,color='k',linestyle='--',label='Setpoint'); ax.axvline(5,color='gray',linestyle=':',label='Disturbance')
    ax.set(xlabel='Time (s)',ylabel='Output',title='Illustrative trial: training seed 0, test seed 100'); ax.legend()
    fig.tight_layout(); fig.savefig(out/'step-response.png',dpi=150); plt.close(fig)
    (out/'config.json').write_text(json.dumps({'dt':DT,'steps':N,'plant':'x_ddot = -a*x_dot - b*x + u + disturbance','action_limit':3,'pid_gains':gains,'episodes_per_seed':args.episodes,'training_seeds':[0,1,2],'validation_seeds':[10,19],'test_seeds':[100,119],'reward':'-(error^2 + 0.03*u^2 + 0.05*velocity^2)','numpy':np.__version__,'python':platform.python_version()},indent=2))
    print(summary.to_string()); print('PID gains:',gains)
if __name__=='__main__':main()