"""Fit an affine local model and a discrete quadratic regulator (NumPy only)."""
import argparse
import json
from pathlib import Path
import numpy as np


def mm(a,b):
    # Explicit contraction avoids spurious Accelerate/NumPy 2 floating-point
    # status warnings on this Mac; all fitted arrays are checked for finiteness.
    return np.einsum('ij,j->i',a,b) if b.ndim==1 else np.einsum('ij,jk->ik',a,b)


def fit(rows):
    X=[];Y=[]
    for r in rows:
        for a,b in zip(r['trace'],r['trace'][1:]):
            x=np.array(a['edge_state']);y=np.array(b['edge_state'])
            if a['time_s']<.10 or abs(x[0])>4 or abs(x[1])>8 or max(abs(x[6:8]))>60:continue
            X.append(np.r_[x,a['edge_action'],1]);Y.append(y[6:12])
    X=np.array(X);Y=np.array(Y)
    scale=np.r_[[2,3,5,5,3,3],[20,20,60,60,30,30],[3,3,2,2]*3,1]
    xs=X/scale
    W=np.linalg.solve(mm(xs.T,xs)+np.eye(25)*.01,mm(xs.T,Y))/scale[:,None]
    A=np.zeros((20,20));B=np.zeros((20,4));c=np.zeros(20)
    A[6:12]=W[:20].T;B[6:12]=W[20:24].T;c[6:12]=W[24]
    A[:6]=.02*A[6:12];A[:6,:6]+=np.eye(6);B[:6]=.02*B[6:12];c[:6]=.02*c[6:12]
    A[12:16,16:20]=np.eye(4);B[16:20]=np.eye(4)
    H=np.zeros((20,8));H[2:6,:4]=np.eye(4);H[12:16,4:]=np.eye(4);H[16:20,4:]=np.eye(4)
    U=np.c_[np.zeros((4,4)),np.eye(4)]
    eq=np.linalg.lstsq(mm(np.eye(20)-A,H)-mm(B,U),c,rcond=None)[0];xeq=mm(H,eq);ueq=mm(U,eq)
    for value in [A,B,c,xeq,ueq]:
        if not np.isfinite(value).all():raise ValueError('Nonfinite fitted model')
    return A,B,c,xeq,ueq,dict(samples=len(X),velocity_fit_rmse=np.sqrt(np.mean((mm(X,W)-Y)**2,axis=0)).tolist(),equilibrium_residual=float(np.linalg.norm(mm(np.eye(20)-A,xeq)-mm(B,ueq)-c)))


def regulator(A,B,angle_weight=50,action_weight=1):
    Q=np.diag([angle_weight,angle_weight, .05,.05,.05,.05, .2,.2,.001,.001,.001,.001]+[.001]*8)
    R=np.diag([1,1,2,2])*action_weight;P=Q.copy()
    for _ in range(5000):
        K=np.linalg.solve(R+mm(mm(B.T,P),B),mm(mm(B.T,P),A))
        new=Q+mm(mm(A.T,P),A-mm(B,K));new=(new+new.T)/2
        if np.max(abs(new-P))<1e-8:P=new;break
        P=new
    K=np.linalg.solve(R+mm(mm(B.T,P),B),mm(mm(B.T,P),A))
    if not np.isfinite(K).all():raise ValueError('Nonfinite regulator')
    return K,float(max(abs(np.linalg.eigvals(A-mm(B,K)))))


def retarget(model,target):
    """Change the desired equilibrium, preserving the identification frame."""
    import copy
    model=copy.deepcopy(model);A=np.array(model['A']);B=np.array(model['B']);c=np.array(model['c'])
    H=np.zeros((20,8));H[2:6,:4]=np.eye(4);H[12:16,4:]=np.eye(4);H[16:20,4:]=np.eye(4)
    U=np.c_[np.zeros((4,4)),np.eye(4)];offset=np.zeros(20);offset[0]=target-32.1
    eq=np.linalg.lstsq(mm(np.eye(20)-A,H)-mm(B,U),c-mm(np.eye(20)-A,offset),rcond=None)[0]
    model.update(x_equilibrium=(mm(H,eq)+offset).tolist(),u_equilibrium=mm(U,eq).tolist(),reference_roll_deg=32.1,target_roll_deg=target)
    return model


def main():
    p=argparse.ArgumentParser();p.add_argument('data');p.add_argument('--output',required=True);p.add_argument('--angle-weight',type=float,default=50);p.add_argument('--action-weight',type=float,default=1);a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    rows=json.loads(Path(a.data).read_text())['rows'];A,B,c,xeq,ueq,metrics=fit(rows[:48]);K,radius=regulator(A,B,a.angle_weight,a.action_weight)
    result=dict(evidence='uncalibrated_simulation',hardware_ready=False,method='Local affine identification and discrete quadratic regulator',
                A=A.tolist(),B=B.tolist(),c=c.tolist(),x_equilibrium=xeq.tolist(),u_equilibrium=ueq.tolist(),gain=K.tolist(),
                fit=metrics,closed_loop_spectral_radius=radius,data=a.data,args=vars(a))
    out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(fit=metrics,radius=radius,head_equilibrium=xeq[2:6].tolist(),u=ueq.tolist())))


if __name__=='__main__':main()
