"""Synthetic full/partial logs and deliberate corruption, never device data."""
import csv,io,math,re,sys,tempfile
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'tools'))
from fast_motion_profile import metadata,parameters,reference
from analyze_fast_motion import analyze_fast
source=(root/'docs/experiment-logs/2026-11-19-motion-sharp60-positive-v161-02.log').read_text().splitlines()
begin=next(i for i,s in enumerate(source) if s.startswith('trace_us,'));columns=source[begin].split(',')
example=next(csv.DictReader(io.StringIO('\n'.join(source[begin:source.index('TRACE_END')]))))
rpm=120;p=parameters(rpm);prefix=[]
for s in source[:begin]:
    if s.startswith(('MOTION_STATS ','ABI_QUALITY ','ABI_QUALITY_RUN ')):continue
    if s.startswith('BENCH_MOTION '):s=metadata(rpm,1)
    if s.startswith('RUN_REFERENCE '):s='RUN_REFERENCE start_angle_rad=0 zero_electric_angle_rad=0 sensor_direction=1 open_field_start_rad=0'
    if s.startswith('ABI_QUALITY_RUN '):s='ABI_QUALITY_RUN invalid=0 duplicate=0 count_delta=7222 encoder_count_delta=7222'
    prefix.append(s)
prefix += [next(s for s in source if s.startswith('ABI_QUALITY ')),
    'ABI_QUALITY_RUN invalid=0 duplicate=0 count_delta=7222 encoder_count_delta=7222',
    'FAST_MOTION_STATUS control_updates=7685 max_control_gap_us=300 fault=0']
def stats(phase,n):return f'MOTION_STATS phase={phase} samples={n} max_error_joint_rad=.0001 error_square=.0001 iq_error_square=0 peak_velocity_motor_rad_s={p["peak_motor_rad_s"]:.6f} vector_limited=0 slew_limited=0 near_iq_limit=0 max_vector_V=.2 vbus_min_V=11.17 vbus_max_V=11.21'
records=[];last=-10000
for t in range(0,p['duration_us'],100):
    q,v,phase=reference(t,1,rpm);period=p['move_us']//160 if phase==1 else 250000//80 if phase==3 else 10000
    if t-last>=period:records.append((t,q,v));last=t
records.append((p['duration_us'],math.radians(p['final_joint_deg']),0))
def make(partial=False,applied_voltage=.2):
    rows=records[:80] if partial else records;rows_text=io.StringIO();writer=csv.DictWriter(rows_text,fieldnames=columns);writer.writeheader()
    for t,q,v in rows:
        r=dict(example);count=round(q*8/(2*math.pi)*4000);angle=count*2*math.pi/4000
        r.update(trace_us=t,loop_us=100,angle_rad=f'{angle:.6f}',raw_rad_s=8*v,filtered_rad_s=8*v,window10ms_rad_s=8*v,
            joint_target_rad=f'{q:.6f}',joint_measured_rad=f'{angle/8:.6f}',joint_velocity_rad_s=f'{v:.6f}',joint_velocity_target_rad_s=f'{v:.6f}',
            Uq_V=applied_voltage,Ud_V=0,iq_target_nominal_A=0,iq_filtered_nominal_A=0,id_filtered_nominal_A=0,bench_flags=0,adc_age_us=100)
        writer.writerow(r)
    pre=list(prefix)
    if partial:
        pre=[s.replace('STOP: bench motion sequence complete','STOP: fast motion tracking exceeded 3deg or 1deg for 20ms').replace('FAST_MOTION_STATUS control_updates=7685 max_control_gap_us=300 fault=0','FAST_MOTION_STATUS control_updates=2000 max_control_gap_us=300 fault=5') for s in pre]
    return '\n'.join(pre+[stats(1,5000),stats(3,0 if partial else 800)])+'\n'+rows_text.getvalue()+'TRACE_END\nHOST final_stop_acknowledged=True\n'
with tempfile.TemporaryDirectory() as tmp:
    path=Path(tmp)/'synthetic.log';full=make();path.write_text(full)
    result=analyze_fast(path);assert result['normal_complete'] and result['advance_candidate'],result
    recovery=full.replace(metadata(rpm,1),metadata(rpm,1,'peak_motor_plus_10pct'))
    path.write_text(recovery);assert analyze_fast(path)['normal_complete']
    path.write_text(recovery.replace('velocity_command_factor=1.1','velocity_command_factor=1.3'))
    try:analyze_fast(path)
    except ValueError:pass
    else:raise AssertionError('Unreviewed recovery factor accepted')
    active07='FAST_VOLTAGE_ACTIVE vector_limit_V=.7000 q_integrator_limit_V=.7000 d_integrator_limit_V=.7000 q_slew_limit_V=.7000 d_slew_limit_V=.7000\n'
    full07=full.replace('vector_limit_V=.5','vector_limit_V=.7').replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=.7')+active07
    path.write_text(full07);result=analyze_fast(path);assert result['voltage_limit_V']==.7 and result['normal_complete']
    full04=full07.replace('iq_limit_nominal_A=.45','iq_limit_nominal_A=.40')+'FAST_CURRENT_ACTIVE iq_limit_nominal_A=.4000\n'
    path.write_text(full04);assert analyze_fast(path)['profile']['iq_limit_nominal_A']==.4
    half_filter=re.sub(r'q_LPF_s=\S+','q_LPF_s=0.0005',full04).replace('current_LPF_s=0.0010','current_LPF_s=0.0005')
    path.write_text(half_filter);assert analyze_fast(path)['normal_complete']
    path.write_text(full04.replace('FAST_CURRENT_ACTIVE iq_limit_nominal_A=.4000','FAST_CURRENT_ACTIVE iq_limit_nominal_A=.4500'))
    try:analyze_fast(path)
    except ValueError:pass
    else:raise AssertionError('Wrong applied current command cap accepted')
    high=make(applied_voltage=.6).replace('max_vector_V=.2','max_vector_V=.6')
    path.write_text(high)
    try:analyze_fast(path)
    except ValueError:pass
    else:raise AssertionError('.6V sample accepted under .5V declared ceiling')
    high07=high.replace('vector_limit_V=.5','vector_limit_V=.7').replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=.7')+active07
    path.write_text(high07);assert analyze_fast(path)['max_vector_V']>.59
    onevolt=make(applied_voltage=.8).replace('max_vector_V=.2','max_vector_V=.8')
    onevolt=onevolt.replace('vector_limit_V=.5','vector_limit_V=1.0').replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=1.0')+active07.replace('.7000','1.0000')
    path.write_text(onevolt);assert analyze_fast(path)['voltage_limit_V']==1. and analyze_fast(path)['max_vector_V']>.79
    path.write_text(onevolt.replace('q_slew_limit_V=1.0000','q_slew_limit_V=.7000'))
    try:analyze_fast(path)
    except ValueError:pass
    else:raise AssertionError('One-volt requested with old slew ceiling accepted')
    for wrong in (full07.replace(active07,''),full07.replace('d_slew_limit_V=.7000','d_slew_limit_V=.5000')):
        path.write_text(wrong)
        try:analyze_fast(path)
        except ValueError:pass
        else:raise AssertionError('Missing or incorrect applied .7V limit accepted')
    path.write_text(make(True));result=analyze_fast(path)
    assert not result['normal_complete'] and not result['advance_candidate'] and result['moving'][1]['FOC_samples']==0 and not result['hold']['reached']
    for bad in (full.replace('control_gap_us=1000','control_gap_us=5000'),full.replace('max_control_gap_us=300','max_control_gap_us=1500'),full.replace('HOST final_stop_acknowledged=True','HOST final_stop_acknowledged=False'),re.sub(r'q_P=\S+','q_P=.9',full)):
        assert bad!=full
        path.write_text(bad)
        try:analyze_fast(path)
        except (ValueError,RuntimeError):pass
        else:raise AssertionError('Corrupt fast log accepted')
print('PASS: synthetic .5/.7V full captures and partial stops audited; missing/wrong applied limits, protection/timing and final-stop corruption rejected')
