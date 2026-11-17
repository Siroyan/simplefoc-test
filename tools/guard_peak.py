"""Parse passive post-stop peak snapshots; never derive a new protection limit."""
import math
import re
from reference_window import comparison_start_us


CONFIG = 'GUARD_PEAK_CONFIG steady_start_us=800000 observation=post_move current_voltage=previous_FOC iq_next=post_move sample_age=velocity_call UART=after_stop'
FIELDS = ('elapsed_us','sample_age_us','adc_age_us','guard_rad_s','control_rad_s','reference_rad_s',
          'iq_applied_nominal_A','iq_next_nominal_A','iq_filtered_nominal_A','Uq_V','Ud_V','correction_nominal_A')


def verify_guard_peak_config(line,steady_start_us=800000):
    expected=CONFIG.replace('steady_start_us=800000',f'steady_start_us={steady_start_us}')
    if line != expected:
        raise RuntimeError('Guard-peak observation timing/configuration mismatch; stopping')


def parse_guard_peaks(lines, required=False):
    configs=[line for line in lines if line.startswith('GUARD_PEAK_CONFIG ')]
    snapshots=[line for line in lines if line.startswith('GUARD_PEAK ')]
    if not configs and not snapshots and not required:
        return None
    if len(configs)!=1:
        raise ValueError('One guard-peak configuration required')
    steady_start_us=comparison_start_us(lines)
    verify_guard_peak_config(configs[0],steady_start_us)
    result={'configuration':configs[0],'steady_start_us':steady_start_us}
    run=next((line for line in lines if line.startswith('RUN_SUMMARY ')),None)
    end_us=float(run.split(': ',1)[1].split(',')[0]) if run else None
    for line in snapshots:
        fields=dict(re.findall(r'([A-Za-z_]+)=([^ ]+)',line))
        scope=fields.get('scope')
        if scope not in ('whole','after_ramp') or scope in result:
            raise ValueError('Unknown or duplicate guard-peak scope')
        if fields.get('valid')=='0':
            result[scope]=None
            continue
        if fields.get('valid')!='1' or any(key not in fields for key in FIELDS):
            raise ValueError('Incomplete guard-peak snapshot')
        values={key:float(fields[key]) for key in FIELDS}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('Nonfinite peak payload retained in raw log; inspect fault before comparing')
        for key in ('elapsed_us','sample_age_us','adc_age_us'):
            if values[key]<0 or values[key]>0xffffffff or values[key]!=int(values[key]):
                raise ValueError('Invalid guard-peak clock/age')
        if end_us is not None and values['elapsed_us']>end_us:
            raise ValueError('Peak recorded after the run stopped')
        if scope=='after_ramp' and values['elapsed_us']<steady_start_us:
            raise ValueError('Post-ramp peak is inside the startup ramp')
        result[scope]=values
    if set(result)!=set(('configuration','steady_start_us','whole','after_ramp')):
        raise ValueError('Both peak scopes required, including explicit unavailable scope')
    if 'STOP: 3-second test complete' in lines and any(result[key] is None for key in ('whole','after_ramp')):
        raise ValueError('Completed run lacks a peak snapshot')
    guard=next((line for line in lines if line.startswith('STOP_GUARD ')),None)
    if guard and result['whole'] is not None:
        peak=float(re.search(r'run_abs_peak_rad_s=([0-9.]+)',guard)[1])
        if abs(abs(result['whole']['guard_rad_s'])-peak)>2e-6:
            raise ValueError('Peak snapshot and independent guard summary disagree')
    if result['after_ramp'] is not None and (result['whole'] is None or abs(result['after_ramp']['guard_rad_s'])>abs(result['whole']['guard_rad_s'])+2e-6):
        raise ValueError('Post-ramp peak exceeds whole-run peak')
    return result
