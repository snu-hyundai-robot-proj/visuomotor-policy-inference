#!/usr/bin/env python3
import csv, json, math, sqlite3
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path('/tmp/replay_actual_trace_right_20260630_022959')
AN = ROOT / 'analysis'
AN.mkdir(parents=True, exist_ok=True)
TICKS = ROOT / 'trace_ticks.jsonl'
FRAMES = ROOT / 'trace_frames.csv'
META = ROOT / 'trace_metadata.json'
BAG = ROOT / 'rosbag' / 'rosbag_0.db3'
JOINTS = [f'j{i}' for i in range(1, 7)]

# HDR35_20 base_link -> tool0 chain.
def rpy_matrix(rpy):
    r,p,y = rpy
    cr,sr = math.cos(r), math.sin(r); cp,sp = math.cos(p), math.sin(p); cy,sy = math.cos(y), math.sin(y)
    Rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]],float)
    Ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]],float)
    Rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]],float)
    return Rz @ Ry @ Rx

def transform(xyz=(0,0,0), rpy=(0,0,0)):
    T=np.eye(4); T[:3,:3]=rpy_matrix(rpy); T[:3,3]=np.asarray(xyz,float); return T

def axis_angle(axis, q):
    axis=np.asarray(axis,float); axis=axis/np.linalg.norm(axis); x,y,z=axis
    c,s=math.cos(q), math.sin(q); C=1-c
    R=np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,y*y*C+c,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
    T=np.eye(4); T[:3,:3]=R; return T

CHAIN=[
    ((0,0,0),(0,0,0),(0,0,1)),
    ((0.15,0.08,0.56),(0,1.5708,-1.5708),(0,0,1)),
    ((0,0.87,-0.04),(0,0,0),(0,0,1)),
    ((0.828,0.175,0.1415),(0,0,0),(1,0,0)),
    ((0.195,0,0.04),(0,0,0),(0,0,1)),
    ((0.175,0,-0.0375),(-1.5708,0,0),(1,0,0)),
]

def fk_deg(qdeg):
    if qdeg is None or np.any(~np.isfinite(qdeg)): return np.array([np.nan,np.nan,np.nan])
    q=np.deg2rad(np.asarray(qdeg,float)); T=np.eye(4)
    for qi,(xyz,rpy,axis) in zip(q, CHAIN): T = T @ transform(xyz,rpy) @ axis_angle(axis, qi)
    return T[:3,3]

def arr(rows, key, width=6):
    out=[]
    for r in rows:
        v=r.get(key)
        if v is None: out.append([np.nan]*width)
        else: out.append([(float(x) if x is not None else np.nan) for x in v[:width]])
    return np.asarray(out,float)

def scalar(rows, key):
    return np.asarray([np.nan if r.get(key) is None else float(r.get(key)) for r in rows], float)

def metrics(a,b,t,frames):
    d=np.asarray(a,float)-np.asarray(b,float); m=np.isfinite(d)
    if not m.any(): return None
    ad=np.abs(d[m]); idxs=np.where(m)[0]; worst_i=idxs[int(np.argmax(np.abs(d[m])))]
    return {
        'mae': float(np.mean(ad)), 'rmse': float(np.sqrt(np.mean(d[m]**2))),
        'p95_abs': float(np.percentile(ad,95)), 'max_abs': float(np.max(ad)),
        'signed_mean': float(np.mean(d[m])),
        'worst_episode_frame': int(frames[worst_i]) if frames is not None and np.isfinite(frames[worst_i]) else None,
        'worst_timestamp_sec': float(t[worst_i]) if t is not None and np.isfinite(t[worst_i]) else None,
    }

def best_lag(a,b,max_lag=100):
    a=np.asarray(a,float); b=np.asarray(b,float); best=None
    for lag in range(-max_lag,max_lag+1):
        if lag < 0: aa=a[-lag:]; bb=b[:len(aa)]
        elif lag > 0: aa=a[:-lag]; bb=b[lag:]
        else: aa=a; bb=b
        m=np.isfinite(aa)&np.isfinite(bb)
        if m.sum()<20: continue
        rmse=float(np.sqrt(np.mean((aa[m]-bb[m])**2)))
        if best is None or rmse<best['rmse']:
            best={'lag_samples':lag,'rmse':rmse,'samples':int(m.sum())}
    return best

def compute_pair_metrics(prefix, A, B, t, frames, joints=range(6)):
    rows=[]; js={}
    for j in joints:
        mm=metrics(A[:,j],B[:,j],t,frames); lag=best_lag(A[:,j],B[:,j])
        rec={'joint':f'j{j+1}', **(mm or {}), 'best_fit_lag_samples': None if lag is None else lag['lag_samples'], 'best_fit_lag_rmse': None if lag is None else lag['rmse']}
        rows.append({'pair':prefix, **rec}); js[f'j{j+1}']=rec
    return rows, js

def load():
    ticks=[json.loads(l) for l in TICKS.open() if l.strip()]
    with FRAMES.open() as f: frames=list(csv.DictReader(f))
    meta=json.loads(META.read_text())
    return ticks,frames,meta

def frame_array(rows,prefix):
    return np.asarray([[float(r.get(f'{prefix}_j{i}', 'nan') or 'nan') for i in range(1,7)] for r in rows],float)

def plot_series(path, title, t, series, labels, ylabel='deg', joints=range(6)):
    joints=list(joints)
    fig,axes=plt.subplots(len(joints),1,figsize=(13,2*len(joints)+1),sharex=True)
    if len(joints)==1: axes=[axes]
    for j,ax in zip(joints,axes):
        for y,label in zip(series,labels): ax.plot(t, y[:,j], lw=1, label=label)
        ax.set_ylabel(f'j{j+1} {ylabel}'); ax.grid(True,alpha=.25); ax.legend(fontsize=7,loc='upper right')
    axes[0].set_title(title); axes[-1].set_xlabel('seconds from replay start')
    fig.tight_layout(); fig.savefig(path,dpi=150); plt.close(fig)

def plot_error(path,title,t,errors,labels,joints=range(6)):
    n=len(list(joints)); fig,axes=plt.subplots(n,1,figsize=(13,2*n+1),sharex=True)
    if n==1: axes=[axes]
    for ax,j in zip(axes,joints):
        for e,label in zip(errors,labels): ax.plot(t,e[:,j],lw=1,label=label)
        ax.axhline(0,color='k',lw=.5); ax.set_ylabel(f'j{j+1} err deg'); ax.grid(True,alpha=.25); ax.legend(fontsize=7,loc='upper right')
    axes[0].set_title(title); axes[-1].set_xlabel('seconds from replay start')
    fig.tight_layout(); fig.savefig(path,dpi=150); plt.close(fig)

def main():
    ticks,frame_rows,meta=load()
    phases=sorted(set(r['phase'] for r in ticks))
    t0=ticks[0]['timestamp_monotonic']; tt=np.asarray([r['timestamp_monotonic']-t0 for r in ticks],float)
    replay=[r for r in ticks if r['phase']=='replay']
    replay_t0=replay[0]['timestamp_monotonic']
    rt=np.asarray([r['timestamp_monotonic']-replay_t0 for r in replay],float)
    rframes=scalar(replay,'episode_frame')
    armA=arr(replay,'arm_action_episode_deg'); armB=arr(replay,'arm_cmd_sent_deg'); armC=arr(replay,'arm_actual_deg')
    handA=arr(replay,'hand_action_episode_deg'); handB=arr(replay,'hand_target_driver_deg'); handC=arr(replay,'hand_actual_deg_raw'); handTJ=arr(replay,'hand_tj_deg')
    ages_arm=scalar(replay,'arm_actual_age_sec'); ages_hand=scalar(replay,'hand_actual_age_sec')
    stale_arm=np.asarray([bool(r.get('arm_actual_stale')) for r in replay]); stale_hand=np.asarray([bool(r.get('hand_actual_stale')) for r in replay])

    # Integrity check.
    md=[]
    md.append('# Trace Integrity Check\n')
    md.append(f'- trace dir: `{ROOT}`')
    md.append(f'- phases: {phases}')
    md.append(f'- replay ticks: {len(replay)}')
    md.append(f'- replay start offset from trace start: {replay_t0 - t0:.6f} sec')
    md.append(f'- comparison plot x-axis: seconds from replay start; tick timing plot x-axis: seconds from trace start.')
    md.append('- actual execution file inspected: `/tmp/drive_arm_hand_replay.py` inside `ros2_teleop_system`')
    md.append('- replay loop: one `synced_step()` call, one `ArmClient.insert()` call, and one `publish_hand_trace()`/hand publish call per replay tick.')
    md.append('- home/ramp/settle: `sync_ramp()` ticks each send one arm insert and one hand publish; the post-home hold loop and settle loop each send one pair per tick. No duplicate send within a single tick was found.')
    md.append('- `/system_right/frame_index` was requested in rosbag, but the topic was not present/published in this run; bag contains hand topics only.')
    (AN/'trace_integrity_check.md').write_text('\n'.join(md)+'\n')

    # Metrics.
    metric_rows=[]; metric_json={'tick_replay':{},'frame_end':{},'freshness':{},'timing':{},'j6':{}}
    for name,A,B in [('arm_A_action_minus_B_command',armA,armB),('arm_B_command_minus_C_actual',armB,armC),('arm_A_action_minus_C_actual',armA,armC)]:
        rows,js=compute_pair_metrics(name,A,B,rt,rframes); metric_rows+=rows; metric_json['tick_replay'][name]=js
    for name,A,B in [('hand_j1_j5_A_action_minus_B_target',handA,handB),('hand_j1_j5_B_target_minus_C_actual',handB,handC),('hand_j1_j5_A_action_minus_C_actual',handA,handC)]:
        rows,js=compute_pair_metrics(name,A,B,rt,rframes,joints=range(5)); metric_rows+=rows; metric_json['tick_replay'][name]=js

    # Frame metrics.
    ft=np.asarray([float(r['episode_time_sec']) for r in frame_rows],float)
    ff=np.asarray([float(r['episode_frame']) for r in frame_rows],float)
    f_armA=frame_array(frame_rows,'arm_action_deg'); f_armB=frame_array(frame_rows,'arm_cmd_frame_end_deg'); f_armC=frame_array(frame_rows,'arm_actual_frame_end_deg')
    f_handA=frame_array(frame_rows,'hand_action_deg'); f_handB=frame_array(frame_rows,'hand_target_deg'); f_handC=frame_array(frame_rows,'hand_actual_frame_end_deg')
    for name,A,B in [('frame_arm_A_action_minus_B_command',f_armA,f_armB),('frame_arm_B_command_minus_C_actual',f_armB,f_armC),('frame_arm_A_action_minus_C_actual',f_armA,f_armC)]:
        rows,js=compute_pair_metrics(name,A,B,ft,ff); metric_rows+=rows; metric_json['frame_end'][name]=js
    for name,A,B in [('frame_hand_j1_j5_A_action_minus_B_target',f_handA,f_handB),('frame_hand_j1_j5_B_target_minus_C_actual',f_handB,f_handC),('frame_hand_j1_j5_A_action_minus_C_actual',f_handA,f_handC)]:
        rows,js=compute_pair_metrics(name,A,B,ft,ff,joints=range(5)); metric_rows+=rows; metric_json['frame_end'][name]=js

    def fresh(x, stale):
        x=x[np.isfinite(x)]
        return {'median':float(np.median(x)), 'p95':float(np.percentile(x,95)), 'max':float(np.max(x)), 'gt_0p05_ratio':float(np.mean(x>0.05)), 'gt_0p2_ratio':float(np.mean(x>0.2)), 'stale_flag_ratio':float(np.mean(stale))}
    metric_json['freshness']['arm_actual_age_sec']=fresh(ages_arm,stale_arm)
    metric_json['freshness']['hand_actual_age_sec']=fresh(ages_hand,stale_hand)

    # Tick timing.
    dt=np.diff(tt)
    metric_json['timing']['all']={'median':float(np.median(dt)), 'p95':float(np.percentile(dt,95)), 'max':float(np.max(dt))}
    for ph in phases:
        pts=np.asarray([r['timestamp_monotonic'] for r in ticks if r['phase']==ph],float)
        dd=np.diff(pts)
        metric_json['timing'][ph]={'count':int(len(pts)), 'median':float(np.median(dd)) if len(dd) else None, 'p95':float(np.percentile(dd,95)) if len(dd) else None, 'max':float(np.max(dd)) if len(dd) else None}
    rdt=np.diff(rt)
    metric_json['timing']['replay'].update({'gt_5ms_ratio':float(np.mean(rdt>0.005)), 'gt_10ms_ratio':float(np.mean(rdt>0.010))})
    counts={int(f):0 for f in range(364)}
    for f in rframes[np.isfinite(rframes)].astype(int): counts[f]=counts.get(f,0)+1
    vals=np.asarray(list(counts.values()))
    metric_json['timing']['replay_frame_tick_count']={'median':float(np.median(vals)), 'min':int(vals.min()), 'max':int(vals.max()), 'frame_0_to_10':{str(i):counts.get(i,0) for i in range(11)}}

    # j6 standalone.
    j6_action=handA[:,5]; j6_eff=handB[:,5]; j6_actual=handC[:,5]
    metric_json['j6']={
        'hold_enabled': True,
        'hold_norm': replay[0].get('j6_hold_norm'),
        'episode_action_deg': {'min':float(np.nanmin(j6_action)), 'max':float(np.nanmax(j6_action)), 'span':float(np.nanmax(j6_action)-np.nanmin(j6_action))},
        'effective_target_deg': {'min':float(np.nanmin(j6_eff)), 'max':float(np.nanmax(j6_eff)), 'span':float(np.nanmax(j6_eff)-np.nanmin(j6_eff))},
        'actual_deg': {'min':float(np.nanmin(j6_actual)), 'max':float(np.nanmax(j6_actual)), 'span':float(np.nanmax(j6_actual)-np.nanmin(j6_actual))},
        'action_vs_effective_is_intentional_override': True,
        'action_minus_effective': metrics(j6_action,j6_eff,rt,rframes),
        'effective_minus_actual': metrics(j6_eff,j6_actual,rt,rframes),
        'action_minus_actual': metrics(j6_action,j6_actual,rt,rframes),
    }

    # Plots.
    plot_series(AN/'arm_action_command_actual.png','Arm action vs replay command vs actual feedback (replay phase)',rt,[armA,armB,armC],['A episode action','B replay command','C actual feedback'])
    plot_error(AN/'arm_tracking_error.png','Arm tracking errors (primary: no time shift)',rt,[armA-armB,armB-armC,armA-armC],['A-B','B-C','A-C'])
    plot_series(AN/'hand_j1_j5_action_target_actual.png','Hand j1-j5 action vs target vs actual (j6 excluded)',rt,[handA,handB,handC],['A episode action','B effective target','C actual feedback'],joints=range(5))
    plot_series(AN/'hand_target_tj_actual_aux.png','Hand auxiliary: effective target vs /joint_states tj/10 vs actual',rt,[handB,handTJ,handC],['B effective target','joint_states tj/10','C actual feedback'])
    plot_error(AN/'hand_tracking_error.png','Hand j1-j5 tracking errors (primary: no time shift)',rt,[handA-handB,handB-handC,handA-handC],['A-B','B-C','A-C'],joints=range(5))
    fig,ax=plt.subplots(figsize=(13,5)); ax.plot(rt,j6_action,label='episode j6 action'); ax.plot(rt,j6_eff,label='effective hold target'); ax.plot(rt,j6_actual,label='actual feedback'); ax.grid(True,alpha=.25); ax.legend(); ax.set_xlabel('seconds from replay start'); ax.set_ylabel('deg'); ax.set_title('j6: episode action vs effective hold target vs actual'); fig.tight_layout(); fig.savefig(AN/'j6_episode_effective_actual.png',dpi=150); plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4)); ax.plot(tt[1:],dt*1000,lw=1); ax.axhline(5,color='g',ls='--',label='5 ms'); ax.axhline(10,color='r',ls='--',label='10 ms'); ax.set_title('Actual tick dt from timestamp_monotonic'); ax.set_xlabel('seconds'); ax.set_ylabel('dt ms'); ax.grid(True,alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(AN/'tick_timing_summary.png',dpi=150); plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,4)); ax.bar(list(counts.keys()), list(counts.values()), width=1); ax.set_title('Replay tick count per episode frame'); ax.set_xlabel('episode_frame'); ax.set_ylabel('tick count'); fig.tight_layout(); fig.savefig(AN/'frame_tick_count.png',dpi=150); plt.close(fig)

    # Frame-level comparison export.
    with (AN/'frame_level_comparison.csv').open('w',newline='') as f:
        fields=['episode_frame','episode_time_sec']
        for p in ['arm_A_action','arm_B_command','arm_C_actual','hand_A_action','hand_B_target','hand_C_actual']:
            fields += [f'{p}_j{i}' for i in range(1,7)]
        fields += ['j6_hold_enabled','j6_action_deg','j6_effective_target_deg','j6_actual_deg']
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader()
        for i,r in enumerate(frame_rows):
            out={'episode_frame':r['episode_frame'],'episode_time_sec':r['episode_time_sec'],'j6_hold_enabled':r['j6_hold_enabled'],'j6_action_deg':r['j6_episode_action_deg'],'j6_effective_target_deg':r['j6_effective_target_deg'],'j6_actual_deg':r['j6_actual_deg']}
            arrays={'arm_A_action':f_armA,'arm_B_command':f_armB,'arm_C_actual':f_armC,'hand_A_action':f_handA,'hand_B_target':f_handB,'hand_C_actual':f_handC}
            for p,a in arrays.items():
                for j in range(6): out[f'{p}_j{j+1}']=a[i,j]
            w.writerow(out)

    # Metrics CSV/JSON.
    with (AN/'tracking_metrics.csv').open('w',newline='') as f:
        fields=['pair','joint','mae','rmse','p95_abs','max_abs','signed_mean','worst_episode_frame','worst_timestamp_sec','best_fit_lag_samples','best_fit_lag_rmse']
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(metric_rows)

    # EEF if freshness is sufficient.
    good=np.isfinite(armC).all(axis=1) & (ages_arm <= 0.2)
    eefA=np.vstack([fk_deg(q) for q in armA]); eefB=np.vstack([fk_deg(q) for q in armB]); eefC=np.vstack([fk_deg(q) if ok else [np.nan,np.nan,np.nan] for q,ok in zip(armC,good)])
    metric_json['eef']={'computed':True,'stale_gap_policy':'arm actual samples with age > 0.2s are NaN/gaps','fresh_ratio':float(np.mean(good)),'model':'HDR35_20 base_link->tool0 j1..j6'}
    for name,xyz in [('episode nominal FK',eefA),('replay command FK',eefB),('actual-feedback FK',eefC)]:
        metric_json['eef'][name]={'start_xyz':xyz[0].tolist(),'end_xyz':xyz[-1].tolist(),'min_xyz':np.nanmin(xyz,axis=0).tolist(),'max_xyz':np.nanmax(xyz,axis=0).tolist()}
    def plot_eef(path,title,series):
        fig,axes=plt.subplots(1,3,figsize=(15,4)); planes=[(0,1,'X','Y'),(0,2,'X','Z'),(1,2,'Y','Z')]
        for ax,(a,b,xl,yl) in zip(axes,planes):
            for xyz,label in series: ax.plot(xyz[:,a],xyz[:,b],label=label,lw=1)
            ax.set_xlabel(xl+' m'); ax.set_ylabel(yl+' m'); ax.grid(True,alpha=.25); ax.legend(fontsize=7)
        fig.suptitle(title); fig.tight_layout(); fig.savefig(path,dpi=150); plt.close(fig)
    plot_eef(AN/'eef_action_command_actual_overlay.png','EEF overlay: episode nominal FK / replay command FK / actual-feedback FK',[(eefA,'episode nominal FK'),(eefB,'replay command FK'),(eefC,'actual-feedback FK')])
    plot_eef(AN/'eef_nominal_action.png','EEF: episode nominal FK',[(eefA,'episode nominal FK')])
    plot_eef(AN/'eef_commanded.png','EEF: replay command FK',[(eefB,'replay command FK')])
    plot_eef(AN/'eef_actual.png','EEF: actual-feedback FK',[(eefC,'actual-feedback FK')])
    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True)
    for k,ax in enumerate(axes):
        ax.plot(rt,eefA[:,k],label='episode nominal FK'); ax.plot(rt,eefB[:,k],label='replay command FK'); ax.plot(rt,eefC[:,k],label='actual-feedback FK'); ax.set_ylabel('XYZ'[k]+' m'); ax.grid(True,alpha=.25); ax.legend(fontsize=7)
    axes[0].set_title('EEF XYZ vs time'); axes[-1].set_xlabel('seconds from replay start'); fig.tight_layout(); fig.savefig(AN/'eef_xyz_vs_time.png',dpi=150); plt.close(fig)
    errBC=eefB-eefC; errAC=eefA-eefC
    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True)
    for k,ax in enumerate(axes):
        ax.plot(rt,errBC[:,k],label='command-actual'); ax.plot(rt,errAC[:,k],label='action-actual'); ax.axhline(0,color='k',lw=.5); ax.set_ylabel('XYZ'[k]+' m'); ax.grid(True,alpha=.25); ax.legend(fontsize=7)
    axes[0].set_title('EEF tracking error XYZ'); axes[-1].set_xlabel('seconds from replay start'); fig.tight_layout(); fig.savefig(AN/'eef_tracking_error_xyz.png',dpi=150); plt.close(fig)
    Path(AN/'eef_summary.json').write_text(json.dumps(metric_json['eef'],indent=2,sort_keys=True))

    # Bag topic note.
    if BAG.exists():
        c=sqlite3.connect(str(BAG)); metric_json['rosbag_topics']={}
        for name,typ,count in c.execute('select topics.name, topics.type, count(messages.id) from topics left join messages on topics.id=messages.topic_id group by topics.id order by topics.name'):
            metric_json['rosbag_topics'][name]={'type':typ,'count':count}

    (AN/'tracking_metrics.json').write_text(json.dumps(metric_json,indent=2,sort_keys=True))
    print(json.dumps({'analysis_dir':str(AN),'replay_ticks':len(replay),'frames':len(frame_rows),'outputs':sorted(p.name for p in AN.iterdir())},indent=2))

if __name__=='__main__': main()
