"""Read-only archive/runtime audit for the paused v176 high-speed checkpoint."""
import argparse,hashlib,json,re,zipfile
from pathlib import Path
from analyze_fast_motion import analyze_fast

def digest(data):return hashlib.sha256(data).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];folder=root/'docs/experiment-logs';records=[];conditions=set()
    for version,rpm in ((169,120),(170,240),(171,240),(172,240),(173,240),(174,360),(175,360),(176,360)):
        log=folder/f'2026-11-20-fast-{rpm}-positive-v{version}-01.log';text=log.read_text(encoding='utf-8')
        result=analyze_fast(log.relative_to(root));saved=json.loads(log.with_suffix('.json').read_text(encoding='utf-8'))
        # Early .5V records predate the explicit duplicate top-level voltage key.
        saved.setdefault('voltage_limit_V',saved['profile']['vector_limit_V'])
        assert result==saved,(version,'analysis changed')
        condition_path=folder/f'2026-11-20-fast-v{version}-condition.json';condition=json.loads(condition_path.read_text(encoding='utf-8'))
        archive=condition_path.with_suffix('.zip');assert digest(archive.read_bytes())==condition['archive_sha256']
        with zipfile.ZipFile(archive) as z:
            for name,sha in condition['source_sha256'].items():assert digest(z.read(name))==sha,(version,name)
            helpers=sorted(n for n in z.namelist() if n.startswith('tools/') and n.endswith('.py'))
            fingerprint=digest(b''.join(z.read(n) for n in helpers))
        assert re.search(r'^HOST python_source_fingerprint=(\w+)',text,re.M)[1]==fingerprint==condition['host_source_fingerprint']
        assert 'HOST final_stop_acknowledged=True' in text and 'TRACE_END' in text
        idle=folder/f'2026-11-20-fast-v{version}-idle-01.log';off=idle.read_text(encoding='utf-8')
        assert 'gate_inputs_low=1' in off and f'FIRMWARE diag_version={version}' in off
        assert 'HOST verified: STOP acknowledged, drive locked until reset, five output-OFF rows' in off
        mode=re.search(r'\bvelocity_limit=(\S+)',next(s for s in text.splitlines() if s.startswith('BENCH_MOTION ')))[1]
        q_lpf=float(re.search(r'\bq_LPF_s=(\S+)',next(s for s in text.splitlines() if s.startswith('CURRENT_ACTIVE ')))[1])
        for name,value in (('vector_limit_V',result['voltage_limit_V']),('iq_limit_nominal_A',result['profile']['iq_limit_nominal_A']),('q_current_LPF_s',q_lpf)):
            if name in condition:assert condition[name]==value
        key=(rpm,result['voltage_limit_V'],result['profile']['iq_limit_nominal_A'],q_lpf,mode)
        assert key not in conditions,'Identical fast condition repeated';conditions.add(key)
        records.append({'version':version,'rpm':rpm,'log':log.name,'sha256':digest(log.read_bytes()),'condition':condition_path.name,
                        'idle_sha256':digest(idle.read_bytes()),'result':result})
    physical=json.loads((folder/'2026-11-20-fast-v176-condition.json').read_text(encoding='utf-8'))
    proposal_path=folder/'2026-11-20-fast-v177-proposal.json';proposal=json.loads(proposal_path.read_text(encoding='utf-8'))
    assert proposal['state']=='built_not_flashed' and proposal['rpm_stage']==600
    with zipfile.ZipFile(proposal_path.with_suffix('.zip')) as z:
        assert digest(proposal_path.with_suffix('.zip').read_bytes())==proposal['archive_sha256']
        for name,sha in proposal['source_sha256'].items():assert digest(z.read(name))==sha
    for name in ('include/BenchHighSpeedMotion.h','include/BenchCurrentGuard.h','include/BoardCurrentObserver.h','include/ObservationGuard.h','include/EdgeVelocity.h','include/BenchMotionStats.h'):
        assert physical['source_sha256'][name]==proposal['source_sha256'][name]
    with zipfile.ZipFile(folder/'2026-11-20-fast-v176-condition.zip') as z:
        assert (root/'src/main.cpp').read_bytes().replace(b'kDiagVersion = 177',b'kDiagVersion = 176')==z.read('src/main.cpp')
    for suffix in ('firmware.elf','firmware.bin'):
        name='.pio/fast-live-build/disco_b_g431b_esc1/'+suffix
        if (root/name).exists():assert digest((root/name).read_bytes())==proposal['source_sha256'][name]
    failed_console=folder/'2026-11-20-fast-v176-paused-idle-error.txt'
    error_bytes=failed_console.read_bytes()
    assert 'could not open port' in error_bytes.decode('utf-16' if error_bytes.startswith(b'\xff\xfe') else 'utf-8',errors='replace')
    data={'state':'paused_by_user','records':records,'normal_complete':sum(r['result']['normal_complete'] for r in records),
          'protective_stops':sum(not r['result']['normal_complete'] for r in records),'last_observed_physical_version':176,
          'last_confirmed_output_OFF_and_locked_log':'2026-11-20-fast-v176-idle-01.log',
          'pause_recheck':'COM10 absent; no commands sent; previous OFF/lock confirmed, present connection/power state unobserved',
          'pause_recheck_error_sha256':digest(failed_console.read_bytes()),'candidate_version':177,'candidate_rpm':600,
          'candidate_state':'built_not_flashed_not_driven','physical_v176_source_ELF_bin_in_condition_archive':True,
          'scope':'Bare motor only; three one-off completed stages, five retained protective stops. Not repeatability, 3733rpm, 70deg/50ms, loaded braking or jump qualification.'}
    with args.output.open('x',encoding='utf-8') as f:json.dump(data,f,indent=2)
    print('PASS: eight captures/archives/runtime fingerprints/final stops; 3 normal, 5 protective; physical v176 preserved, v177 built only, pause USB recheck failure retained')

if __name__=='__main__':main()
