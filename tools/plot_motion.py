"""Standalone figures from raw post-stop RAM traces; no device access."""
import argparse,csv,io,math,re
from pathlib import Path
from motion_profile import fields,reference

def main():
    p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('--output',required=True);args=p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    source=Path(args.log);lines=source.read_text(encoding='utf-8').splitlines()
    meta=fields(next(s for s in lines if s.startswith('BENCH_MOTION ')))
    profile=fields(next(s for s in lines if s.startswith('BENCH_PROFILE ')))
    first=next(i for i,s in enumerate(lines) if s.startswith('trace_us,'));last=lines.index('TRACE_END',first)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO('\n'.join(lines[first:last])))]
    t=[r['trace_us']/1e6 for r in rows];degree=180/math.pi;rpm=30/math.pi
    planned=[reference(r['trace_us'],int(meta['direction']),int(meta['rpm']),int(meta['shape'])) for r in rows]
    q=[r['joint_measured_rad']*degree for r in rows];qr=[a[0]*degree for a in planned]
    fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True,constrained_layout=True)
    ax=axes[0,0];ax.plot(t,qr,'--',label='reference');ax.plot(t,q,label='ABI motor / 8');ax.set_ylabel('Virtual joint (deg)');ax.legend()
    ax=axes[0,1];ax.plot(t,[a-b for a,b in zip(qr,q)],label='reference - ABI');ax.axhline(.5,color='grey',linestyle=':');ax.axhline(-.5,color='grey',linestyle=':');ax.set_ylabel('Tracking error (deg)')
    ax=axes[1,0];ax.plot(t,[a[1]*8*rpm for a in planned],'--',label='trajectory');ax.plot(t,[r['joint_velocity_target_rad_s']*8*rpm for r in rows],label='controller command',alpha=.65);ax.plot(t,[r['filtered_rad_s']*rpm for r in rows],label='feedback');ax.set_ylabel('Motor speed (rpm)');ax.legend(fontsize=8)
    ax=axes[1,1];ax.plot(t,[r['iq_target_nominal_A'] for r in rows],label='applied Iq command');ax.plot(t,[r['iq_filtered_nominal_A'] for r in rows],label='measured Iq');limit=profile['iq_limit_nominal_A'];ax.axhline(limit,color='grey',linestyle=':');ax.axhline(-limit,color='grey',linestyle=':');ax.set_ylabel('Iq (nominal A)');ax.legend(fontsize=8)
    ax=axes[2,0];ax.plot(t,[math.hypot(r['Uq_V'],r['Ud_V']) for r in rows],label='command sqrt(Uq^2+Ud^2)');ax.axhline(.5,color='grey',linestyle=':',label='limit');ax.set_ylim(0,.55);ax.set_ylabel('Voltage command (V)');ax.legend(fontsize=8)
    ax=axes[2,1];ax.plot(t,[r['vbus_adc']*3.3/4096*187/18 for r in rows]);ax.set_ylabel('ADC Vbus (V)')
    for ax in axes.flat:ax.grid(True,alpha=.25);ax.set_xlim(t[0],t[-1])
    for ax in axes[-1]:ax.set_xlabel('Time from first FOC (s)')
    stop=next(s for s in lines if s.startswith('STOP: '));version=re.search(r'FIRMWARE diag_version=(\d+)','\n'.join(lines))[1]
    fig.suptitle(f'Bare motor v{version}: {meta["rpm"]:g} rpm reference, shape {meta["shape"]:g}, direction {meta["direction"]:g}\n{stop}; same-ABI tracking, nominal current scaling',fontsize=11)
    out=Path(args.output)
    if out.exists():raise FileExistsError(out)
    fig.savefig(out,dpi=160);plt.close(fig)
    print(out)

if __name__=='__main__':main()
