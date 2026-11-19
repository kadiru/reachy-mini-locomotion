"""Compare gait progress over the same 6.6 s window; no policy training.

Historical torque-only substitutions isolate one model change. Other cases
expose combined changes or individual assumptions in the newer model. None
establish a physical actuator envelope, thermal limit, or hardware readiness.
"""
import argparse
import copy
import json
import math
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import SwaySimulator,Gait
from reachy_locomotion.contact_probe import MODEL_PATH

CACHE={}


def comparison_window(r,seconds=6.6):
    rows=[x for x in r['trajectory'] if x['measuring']]
    n=round(seconds*50)+1
    if len(rows)<n:
        return {'complete':False,'reason':r['failure_reason'],'available_samples':len(rows)}
    rows=rows[:n]
    p=np.array([x['foot_xyz_m'] for x in rows]);yaw=rows[0]['foot_rpy_rad'][2]
    rot=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    p=(p-p[0])@rot
    headings=np.unwrap([x['foot_rpy_rad'][2] for x in rows]);headings-=headings[0]
    duration=rows[-1]['time_s']-rows[0]['time_s']
    if not math.isclose(duration,seconds,abs_tol=1e-8):raise ValueError('Incorrect comparison duration')
    acc=np.diff(p,n=2,axis=0)/.02**2
    return {'complete':True,'measured_seconds':duration,
            'forward_mm':float(1000*p[-1,0]),'lateral_mm':float(1000*p[-1,1]),
            'forward_mm_s':float(1000*p[-1,0]/duration),
            'heading_change_deg':float(np.degrees(headings[-1])),
            'rms_position_difference_acceleration_m_s2':float(np.sqrt(np.mean(acc**2))),
            'path_10hz_m':p[::5].tolist()}


def run(payload):
    label,gname,c,g=payload
    key=json.dumps(c,sort_keys=True)
    if key not in CACHE:CACHE[key]=SwaySimulator('experiments/upstream-reachy-mini',c)
    gait=Gait(**g);period=round(50/gait.frequency_hz)/50
    r=CACHE[key].run(gait,cycles=math.ceil(6.6/period-1e-9),record=True)
    window=comparison_window(r)
    del r['trajectory']
    return {'model_case':label,'gait_name':gname,'comparison_window':window,'full_cycle_rollout':r}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output)
    if out.exists():parser.error('Choose a new report')
    legacy=json.loads(Path('configs/sway-cad-proxy.json').read_text())
    current=json.loads(Path('configs/motors-datasheet.json').read_text())
    cases={'historical_1p2Nm':legacy}
    for cap in [.6,.416]:
        c=copy.deepcopy(legacy);c['motor_force_limit_nm']=cap
        cases['historical_torque_only_'+str(cap)]=c
    cases['current_5V']=current
    for v in [5.,6.]:
        c=copy.deepcopy(current);c['motor_model']['voltage_v']=v
        if v==6.:cases['current_6V']=copy.deepcopy(c)
        c['motor_model'].update(torque_derating=1.,pwm_limit_fraction=1.)
        cases['current_'+str(v)+'V_full_peak_not_continuous']=c
    source=ET.parse(Path('experiments/upstream-reachy-mini')/MODEL_PATH).getroot()
    cad_mass=sum(float(i.get('mass')) for i in source.find('worldbody').iter('inertial'))
    c=copy.deepcopy(current);c['motor_model']['total_mass_kg']=cad_mass
    cases['current_CAD_mass_only']=c
    c=copy.deepcopy(current);c['motor_model'].update(target_speed_rad_s=8.,target_acceleration_rad_s2=120.)
    cases['current_relaxed_command_rates_only']=c
    gaits={name:json.loads(Path(path).read_text()) for name,path in {
        'historical_fast':'configs/sway-candidate.json',
        'initial_trained':'experiments/reports/motor-training-2026-11-18-policy.json',
        'smooth_trained':'configs/sway-trained.json'}.items()}
    jobs=[]
    for label,c in cases.items():
        c=copy.deepcopy(c)
        c['notes']='Diagnostic case '+label+'; numeric fields specify this variant. See the comparison report for interpretation.'
        for name,g in gaits.items():
            for dt in [.002,.001]:
                c=copy.deepcopy(c);c['timestep_s']=dt;jobs.append((label,name,c,g))
    results=[]
    receipt={'evidence':'uncalibrated_simulation','hardware_ready':False,
        'purpose':'matched_duration_diagnostic_not_new_training',
        'measurement_seconds':6.6,'warmup':'two cycles for each gait, excluded',
        'warning':'full_peak cases are momentary datasheet bounds, not continuous ratings',
        'completed':False,'results':results}
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(run,jobs):
            results.append(r)
            out.write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
            w=r['comparison_window']
            print(r['model_case'],r['gait_name'],r['full_cycle_rollout']['config']['timestep_s'],
                  round(w.get('forward_mm',0),2),round(w.get('lateral_mm',0),2),
                  round(w.get('heading_change_deg',0),2),w['complete'],flush=True)
    receipt['completed']=True
    out.write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
