"""Metadata checks for the separately selected reference-following speed range."""
import math
import re
from reference_window import comparison_start_us

GUARD_CONFIG='SPEED_GUARD_CONFIG mode=reference_envelope deviation_rad_s=2 hard_abs_rad_s=8.5 max_target_rad_s=6.283185 source=original_window2ms_LPF2ms signed_motor_velocity=1'
STATS_CONFIG='VELOCITY_STATS_CONFIG period_us=2000 start_us=2000000 source=ABI_interval frame=motor_signed samples=post_ramp_only UART=after_stop'


def verify_speed_guard_config(line):
    if line!=GUARD_CONFIG:
        raise RuntimeError('Reference-envelope guard configuration mismatch; stopping')


def verify_velocity_stats_config(line,start_us=2000000):
    if start_us not in (800000,2000000) or line!=STATS_CONFIG.replace('start_us=2000000',f'start_us={start_us}'):
        raise RuntimeError('High-rate velocity statistics configuration mismatch; stopping')


def parse_interval_velocity_stats(lines):
    guards=[line for line in lines if line.startswith('SPEED_GUARD_CONFIG ')]
    configs=[line for line in lines if line.startswith('VELOCITY_STATS_CONFIG ')]
    summaries=[line for line in lines if line.startswith('VELOCITY_STATS ')]
    if not guards and not configs and not summaries:
        return None
    if len(guards)!=1 or len(configs)!=1 or len(summaries)!=1:
        raise ValueError('One complete higher-range guard/statistics configuration and summary required')
    start_us=comparison_start_us(lines) if any(l.startswith('REFERENCE_RAMP ') for l in lines) else 2000000
    verify_speed_guard_config(guards[0]);verify_velocity_stats_config(configs[0],start_us)
    values={key:float(value) for key,value in re.findall(r'([A-Za-z_]+)=([^ ]+)',summaries[0])}
    complete='STOP: 3-second test complete' in lines
    run=next((line for line in lines if line.startswith('RUN_SUMMARY ')),None)
    if complete and run is None:
        raise ValueError('Completed higher-range run lacks its actual stop time')
    run_end=float(run.split(': ',1)[1].split(',')[0]) if run else 3010000
    if values.get('valid')==0:
        if complete:
            raise ValueError('Completed higher-range run has no post-ramp speed statistics')
        return {'available':False,'complete_run':False,'scope':'no_post_ramp_samples','raw_summary':summaries[0]}
    required=('valid','invalid','samples','first_elapsed_us','last_elapsed_us','covered_us','displacement_rad',
              'mean_motor_rad_s','rms_error_rad_s','minimum_rad_s','maximum_rad_s','slow_fraction','reverse_fraction')
    if any(key not in values for key in required) or not all(math.isfinite(value) for value in values.values()):
        raise ValueError('Incomplete or nonfinite interval statistics')
    if values['valid']!=1 or values['invalid']!=0 or values['samples']<=0 or values['covered_us']<=0:
        raise ValueError('Invalid interval statistics state')
    if values['first_elapsed_us']<start_us or values['last_elapsed_us']>run_end or abs(values['last_elapsed_us']-values['first_elapsed_us']-values['covered_us'])>1:
        raise ValueError('Interval statistics coverage mismatch')
    if not 2000<=values['covered_us']/values['samples']<=20000 or (complete and values['covered_us']<3000000-start_us-50000):
        raise ValueError('Incomplete post-ramp interval coverage or cadence')
    # Displacement is printed to six decimals. Its half-unit rounding error
    # becomes significant when a protection stop leaves only a short interval.
    mean_rounding_tolerance=1e-5+0.5/values['covered_us']
    if abs(values['displacement_rad']*1e6/values['covered_us']-values['mean_motor_rad_s'])>mean_rounding_tolerance:
        raise ValueError('Mean velocity and interval displacement disagree')
    if values['rms_error_rad_s']<0 or not values['minimum_rad_s']-1e-5<=values['mean_motor_rad_s']<=values['maximum_rad_s']+1e-5:
        raise ValueError('Invalid interval moments')
    if not all(0<=values[key]<=1 for key in ('slow_fraction','reverse_fraction')):
        raise ValueError('Invalid interval duration fraction')
    values['relative_rms_error']=values['rms_error_rad_s']/6.283185
    return {'available':True,'complete_run':complete,
            'scope':'full_post_ramp_interval' if complete else 'partial_post_ramp_until_stop',
            'configuration':configs[0],'guard_configuration':guards[0],**values}
