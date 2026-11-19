"""Archive exact firmware and all Python helpers before a fast-motion trial."""
import argparse,hashlib,json,re,zipfile
from datetime import datetime
from pathlib import Path
from fast_motion_profile import parameters

def main():
    p=argparse.ArgumentParser();p.add_argument('--version',type=int,required=True);p.add_argument('--rpm',type=int,required=True)
    p.add_argument('--vector-limit',type=float,choices=(.5,.7,1.),default=.5);p.add_argument('--proposal',action='store_true')
    p.add_argument('--iq-limit',type=float,choices=(.4,.45),default=.45)
    p.add_argument('--q-lpf',type=float,choices=(.001,.0005),default=.001)
    p.add_argument('--velocity-limit',choices=('peak_motor','peak_motor_plus_10pct'),default='peak_motor')
    p.add_argument('--build-dir',type=Path,default=Path('.pio/observer-build'));args=p.parse_args()
    root=Path(__file__).resolve().parents[1];parameters(args.rpm)
    if int(re.search(r'kDiagVersion = (\d+)',(root/'src/main.cpp').read_text(encoding='utf-8'))[1])!=args.version:raise ValueError('Source version mismatch')
    date=datetime.now().astimezone().strftime('%Y-%m-%d');folder=root/'docs/experiment-logs';stem=f'{date}-fast-v{args.version}-'+('proposal' if args.proposal else 'condition')
    files=[root/'src/main.cpp',root/'platformio.ini']+sorted((root/'include').glob('*.h'))+sorted((root/'tools').glob('*.py'))
    files += [root/args.build_dir/'disco_b_g431b_esc1'/n for n in ('firmware.elf','firmware.bin')]
    hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    archive=folder/(stem+'.zip')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for path in files:z.write(path,path.relative_to(root).as_posix())
    data={'version':args.version,'rpm_stage':args.rpm,'trajectory':parameters(args.rpm),'supply_V':11.1,'input_current_limit_A':.8,'vector_limit_V':args.vector_limit,'iq_limit_nominal_A':args.iq_limit,'q_current_LPF_s':args.q_lpf,
        'state':'built_not_flashed' if args.proposal else 'firmware_condition','velocity_command_mode':args.velocity_limit,
        'build_flags':f'-DDIAG_PWM_HZ=12500 -DHAL_OPAMP_MODULE_ENABLED -DBENCH_HIGH_SPEED_RPM={args.rpm}'+(' -DBENCH_FAST_VOLTAGE_LIMIT=1.0f' if args.vector_limit==1. else ' -DBENCH_FAST_VOLTAGE_LIMIT=.7f' if args.vector_limit==.7 else ''),
        'source_sha256':hashes,'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
        'host_source_fingerprint':hashlib.sha256(b''.join(p.read_bytes() for p in sorted((root/'tools').glob('*.py')))).hexdigest()}
    with (folder/(stem+'.json')).open('x',encoding='utf-8') as f:json.dump(data,f,indent=2)
    print('Archived exact firmware/headers/host helpers and conditions:',stem)

if __name__=='__main__':main()
