"""Compare repeated known-probe velocity fits without approving a correction phase."""
import argparse
import cmath
import glob
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('patterns',nargs='+');parser.add_argument('--output',type=Path)
    args=parser.parse_args();rows=[]
    for path in sorted({Path(p) for pattern in args.patterns for p in glob.glob(pattern)}):
        x=json.loads(subprocess.check_output([sys.executable,str(Path(__file__).with_name('analyze_current_probe.py')),str(path)],text=True))
        if not x['stop_acknowledged'] or not x['closed_loop_velocity_response']:
            raise ValueError(f'Completed velocity-response capture required: {path}')
        config={k:float(v) for k,v in re.findall(r'([A-Za-z_]+)=([0-9.]+)',x['probe'])}
        rows.append({'log':path.name,'frequency_Hz':config['frequency_Hz'],'probe_phase_rad':config.get('phase_rad',0),
                     'excitation_unclipped':x['excitation_unclipped'],
                     'filtered':x['closed_loop_velocity_response']['filtered_rad_s'],
                     'window10ms':x['closed_loop_velocity_response']['window10ms_rad_s'],
                     'split_filtered':[p['filtered_rad_s'] for p in x['split_velocity_responses']]})
    groups=[]
    for frequency in sorted({r['frequency_Hz'] for r in rows}):
        selected=[r for r in rows if r['frequency_Hz']==frequency]
        components=[cmath.rect(r['filtered']['gain_rad_s_per_nominal_A'],math.radians(r['filtered']['phase_deg'])) for r in selected]
        mean=sum(components)/len(components)
        split_spreads=[abs(math.remainder(r['split_filtered'][1]['phase_deg']-r['split_filtered'][0]['phase_deg'],360)) for r in selected]
        groups.append({'frequency_Hz':frequency,'runs':len(selected),
                       'mean_coherent_gain_rad_s_per_nominal_A':abs(mean),'mean_coherent_phase_deg':math.degrees(cmath.phase(mean)),
                       'individual_phase_range_deg':[min(r['filtered']['phase_deg'] for r in selected),max(r['filtered']['phase_deg'] for r in selected)],
                       'max_within_run_split_phase_difference_deg':max(split_spreads),
                       'unclipped_runs':sum(r['excitation_unclipped'] for r in selected),
                       'sine_R2_range':[min(r['filtered']['coherent_velocity_component']['sine_R2_after_removing_linear_trend'] for r in selected),max(r['filtered']['coherent_velocity_component']['sine_R2_after_removing_linear_trend'] for r in selected)],
                       'leave_one_run_out_phases_deg':[math.degrees(cmath.phase((sum(components)-c)/(len(components)-1))) for c in components] if len(components)>2 else None})
    result={'groups':groups,'runs':rows,'note':'Complex means combine independently phase-normalized responses. Original rotor ripple, changing initial position, feedback and clipping can bias these short records. Split differences and repeat scatter remain essential. No compensation phase or open-loop motor parameter is approved by this report.'}
    output=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(output,encoding='utf-8')
    print(output)


if __name__=='__main__':main()
