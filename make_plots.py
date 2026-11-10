"""Plot saved training returns and paired representative response/control signals."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from main import rollout
root=Path('results');config=json.loads((root/'config.json').read_text())
returns=pd.read_csv(root/'training_returns.csv')
fig,ax=plt.subplots(figsize=(8,4))
for c in returns:
 ax.plot(returns[c].rolling(50,min_periods=1).mean(),label=c)
ax.set(xlabel='Training episode',ylabel='Return (50-episode moving average)',title='Learning curves: all training seeds');ax.legend()
fig.tight_layout();fig.savefig(root/'learning-curves.png',dpi=150);plt.close(fig)
q=np.load(root/'q_table_seed_0.npy')
fig,axes=plt.subplots(3,1,figsize=(8,8),sharex=True)
for method in ['PID','Q-learning']:
 z,_=rollout(method,100,q=q,gains=config['pid_gains'])
 for ax,col in zip(axes,[1,4,3]): ax.plot(z[:,0],z[:,col],label=method)
axes[0].axhline(1,color='k',linestyle='--');axes[0].set_ylabel('Output')
axes[1].set_ylabel('Tracking error');axes[2].set(ylabel='Control input',xlabel='Time (s)')
for ax in axes: ax.axvline(5,color='gray',linestyle=':');ax.legend()
fig.suptitle('Representative trial: policy seed 0, test seed 100, nominal plant')
fig.tight_layout();fig.savefig(root/'response-error-control.png',dpi=150);plt.close(fig)
print('Saved learning-curves.png and response-error-control.png')
