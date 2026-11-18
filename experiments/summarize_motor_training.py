"""Summarize independent evaluation; numerical validity is not walking success."""
import argparse
import json
import math
from pathlib import Path



def qualified(r):
    return (r['valid'] and r['forward_m_per_s']>=.001
            and abs(r['lateral_m'])<=.3*r['forward_m']+.002
            and abs(r['heading_change_rad'])<=math.radians(10)
            and r['max_foot_tilt_deg']<=12)


def summarize(receipt):
    if not receipt.get('completed'):raise ValueError('Evaluation is not complete')
    grouped={}
    for item in receipt['evaluation']:
        grouped.setdefault(item['label'],[]).append(item)
    result={'evidence':'uncalibrated_simulation','hardware_ready':False,
      'qualification':'valid; >=1 mm/s forward; lateral <=30% forward +2 mm; |heading| <=10 deg; tilt <=12 deg',
      'qualification_is_not_hardware_acceptance':True,'labels':{}}
    for label,items in grouped.items():
        held=[i['result'] for i in items if i['domain_index']>0]
        nominal=[i['result'] for i in items if i['domain_index']==0]
        summary={'heldout_cases':len(held),'heldout_valid':sum(r['valid'] for r in held),
                 'heldout_forward_over_1mm':sum(r['forward_m']>.001 for r in held),
                 'heldout_qualified':sum(qualified(r) for r in held),
                 'heldout_speed_mm_s_range':[min(r['forward_m_per_s'] for r in held)*1000,max(r['forward_m_per_s'] for r in held)*1000],
                 'nominal':[{k:r[k] for k in ['actual_frequency_hz','measured_seconds','forward_m','lateral_m',
                     'heading_change_rad','rms_base_acceleration_m_s2','rms_base_angular_velocity_rad_s',
                     'max_foot_tilt_deg','max_head_orientation_tracking_error_deg','left_unloaded_fraction',
                     'right_unloaded_fraction','any_motor_torque_saturated_fraction']}|{'dt_s':r['config']['timestep_s'],'qualified':qualified(r)} for r in nominal]}
        pairs=[]
        for di in sorted(set(i['domain_index'] for i in items)):
            a,b=[i['result'] for i in items if i['domain_index']==di]
            pairs.append({'domain_index':di,'forward_difference_mm':abs(a['forward_m']-b['forward_m'])*1000,
                          'heading_difference_deg':abs(a['heading_change_rad']-b['heading_change_rad'])*180/math.pi,
                          'converged_at_20pct_plus_1mm':abs(a['forward_m']-b['forward_m'])<=.2*max(abs(a['forward_m']),abs(b['forward_m']))+.001})
        summary['timestep_pairs']=pairs
        result['labels'][label]=summary
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report');p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output)
    if out.exists():p.error('Choose a new summary path')
    out.write_text(json.dumps(summarize(json.loads(Path(a.report).read_text())),indent=2,allow_nan=False)+'\n')
    print(out)


if __name__=='__main__':main()
