"""Freeze the exact built motion firmware and sources before the next build."""
import argparse,hashlib,json,re,shutil
from datetime import datetime
from pathlib import Path
def main():
    p=argparse.ArgumentParser();p.add_argument('--version',type=int,required=True);p.add_argument('--rpm',type=int,required=True);p.add_argument('--shape',type=int,required=True)
    p.add_argument('--date',default=datetime.now().astimezone().strftime('%Y-%m-%d'));args=p.parse_args();datetime.strptime(args.date,'%Y-%m-%d')
    root=Path(__file__).resolve().parents[1];s=(root/'src/main.cpp').read_text(encoding='utf-8')
    if int(re.search(r'kDiagVersion = (\d+)',s)[1])!=args.version:raise ValueError('Source version mismatch')
    target=root/f'.pio/motion-v{args.version}-snapshot';target.mkdir(exist_ok=False)
    files=['src/main.cpp','include/BenchMotion.h','include/BenchMotionStats.h','include/BenchJoint.h','include/BenchCurrentGuard.h','include/BoundedBLDCMotor.h','include/TrackingCurrentPI.h','include/TraceSample.h','include/BoardCurrentObserver.h','include/ObservationGuard.h','.pio/observer-build/disco_b_g431b_esc1/firmware.elf','.pio/observer-build/disco_b_g431b_esc1/firmware.bin','tools/capture_motion.py','tools/motion_profile.py','tools/analyze_motion.py','tools/capture_test.py','tools/bench_metadata.py','tools/analyze_bench.py']
    hashes={}
    for name in files:
        path=root/name;hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest();shutil.copy2(path,target/path.name)
    record={'version':args.version,'rpm':args.rpm,'shape':args.shape,'supply_V':11.1,'input_current_limit_A':.8,
            'build_flags':f'-DDIAG_PWM_HZ=12500 -DHAL_OPAMP_MODULE_ENABLED -DBENCH_MOTION_RPM={args.rpm} -DBENCH_MOTION_SHAPE={args.shape}',
            'snapshot_directory':str(target.relative_to(root)),'sha256':hashes}
    with (root/f'docs/experiment-logs/{args.date}-motion-v{args.version}-condition.json').open('x',encoding='utf-8') as f:json.dump(record,f,indent=2)
    print('Saved exact source/header/ELF/bin snapshot and condition hashes')
if __name__=='__main__':main()
