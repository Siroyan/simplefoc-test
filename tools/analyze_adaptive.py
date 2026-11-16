"""Validate bounded adaptive diagnostics and report common-window learning intervals."""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import re


def metadata(line):
    return {key:float(value) for key,value in re.findall(r'([A-Za-z_]+)=(-?[0-9.]+)',line)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();lines=args.log.read_text(encoding='utf-8').splitlines()
    if 'HOST final_stop_acknowledged=True' not in lines:raise ValueError('Completed final stop acknowledgement required')
    config=metadata(next(l for l in lines if l.startswith('ADAPTIVE_CONFIG ')))
    summary=metadata(next(l for l in lines if l.startswith('ADAPTIVE_SUMMARY ')))
    if summary['invalid'] or summary['peak_nominal_A']>.030001 or math.hypot(summary['cosine_nominal_A'],summary['sine_nominal_A'])>.030001:
        raise ValueError('Invalid adaptive state or excessive correction')
    a=next(n for n,l in enumerate(lines) if l.startswith('ADAPTIVE_SAMPLES '));samples=[]
    for line in lines[a+1:]:
        parts=line.split(',')
        if len(parts)!=4 or not parts[0].isdigit():break
        elapsed,pid,correction,total=map(float,parts)
        if not all(math.isfinite(v) for v in (elapsed,pid,correction,total)) or abs(correction)>.030001 or abs(total)>.450001:
            raise ValueError('Invalid or unbounded adaptive sample')
        if elapsed<=800000 and abs(correction)>1e-6:raise ValueError('Correction before gate')
        if abs(total-max(-.45,min(.45,pid+correction)))>3e-6:raise ValueError('Correction/mix telemetry does not agree')
        samples.append([elapsed,pid,correction,total])
    if not samples:raise ValueError('No adaptive samples')
    if config['learning_rate']==0 and (summary['peak_nominal_A']!=0 or any(r[2]!=0 for r in samples)):
        raise ValueError('Zero-learning comparison produced correction')
    a=next(n for n,l in enumerate(lines) if l.startswith('trace_us,'));b=lines.index('TRACE_END',a)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO('\n'.join(lines[a:b])))]
    intervals=[]
    for start,end in ((.8,1.5),(1.5,2.2),(2.2,3.01),(.8,3.01)):
        selected=[r for r in rows if start*1e6<=r['trace_us']<=end*1e6]
        if len(selected)<2:continue
        duration=selected[-1]['trace_us']-selected[0]['trace_us']
        square=sum((y['trace_us']-x['trace_us'])*(x['window10ms_rad_s']-x['target_rad_s'])**2 for x,y in zip(selected,selected[1:]))
        intervals.append({'actual_start_us':selected[0]['trace_us'],'actual_end_us':selected[-1]['trace_us'],
                          'window10ms_tracking_rms_rad_s':math.sqrt(square/duration)})
    output=json.dumps({'log':args.log.name,'config':config,'summary':summary,'bounded_samples':samples,
                       'learning_intervals':intervals,'note':'Intervals diagnose within-run evolution; the primary comparison still uses the common full interval after .8s. Samples are nominal commands, not calibrated current or CV measurements.'},indent=2)+'\n'
    if args.output:args.output.write_text(output,encoding='utf-8')
    print(output)


if __name__=='__main__':main()
