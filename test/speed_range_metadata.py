"""Reject weakened guards, missing high-rate data, and inconsistent units/coverage."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from speed_range import GUARD_CONFIG,STATS_CONFIG,parse_interval_velocity_stats,verify_speed_guard_config

summary='VELOCITY_STATS valid=1 invalid=0 samples=499 first_elapsed_us=2000100 last_elapsed_us=2998100 covered_us=998000 displacement_rad=6.2874 mean_motor_rad_s=6.3 rms_error_rad_s=.4 minimum_rad_s=5 maximum_rad_s=7 slow_fraction=0 reverse_fraction=0'
lines=[GUARD_CONFIG,STATS_CONFIG,summary,'RUN_SUMMARY elapsed_us,displacement_rad: 2999000,12.58','STOP: 3-second test complete']
assert parse_interval_velocity_stats(lines)['available']
assert parse_interval_velocity_stats(lines)['complete_run']
assert parse_interval_velocity_stats(lines)['scope']=='full_post_ramp_interval'
fast_reference='REFERENCE_RAMP duration_us=800000 target_rad_s=6.283185 trace_target=final_goal origin=trace_start'
fast_summary=summary.replace('samples=499','samples=1099').replace('first_elapsed_us=2000100','first_elapsed_us=800100').replace('covered_us=998000','covered_us=2198000').replace('displacement_rad=6.2874','displacement_rad=13.8474')
fast=[GUARD_CONFIG,STATS_CONFIG.replace('2000000','800000'),fast_summary,fast_reference,'RUN_SUMMARY elapsed_us,displacement_rad: 2999000,16.3','STOP: 3-second test complete']
assert parse_interval_velocity_stats(fast)['complete_run']
short_summary='VELOCITY_STATS valid=1 invalid=0 samples=13 first_elapsed_us=800023 last_elapsed_us=826673 covered_us=26650 displacement_rad=0.166504 mean_motor_rad_s=6.247819 rms_error_rad_s=1.313763 minimum_rad_s=3.844339 maximum_rad_s=8.558035 slow_fraction=0 reverse_fraction=0'
short=[GUARD_CONFIG,STATS_CONFIG.replace('2000000','800000'),short_summary,fast_reference,'RUN_SUMMARY elapsed_us,displacement_rad: 827918,2.508562','STOP: reference speed envelope exceeded']
assert parse_interval_velocity_stats(short)['scope']=='partial_post_ramp_until_stop'
try:parse_interval_velocity_stats([l.replace('mean_motor_rad_s=6.247819','mean_motor_rad_s=6.248819') for l in short])
except ValueError:pass
else:raise AssertionError('Short-interval mean mismatch accepted beyond decimal rounding')
try:parse_interval_velocity_stats([l.replace('samples=1099','samples=499').replace('last_elapsed_us=2998100','last_elapsed_us=1798100').replace('covered_us=2198000','covered_us=998000').replace('displacement_rad=13.8474','displacement_rad=6.2874') for l in fast])
except ValueError:pass
else:raise AssertionError('Fast-ramp completion accepted less than its full post-ramp interval')
partial_summary=summary.replace('samples=499','samples=100').replace('2998100','2200100').replace('covered_us=998000','covered_us=200000').replace('displacement_rad=6.2874','displacement_rad=1.26')
partial=[GUARD_CONFIG,STATS_CONFIG,partial_summary,'RUN_SUMMARY elapsed_us,displacement_rad: 2201000,8','STOP: reference speed envelope exceeded']
assert parse_interval_velocity_stats(partial)['available']
assert not parse_interval_velocity_stats(partial)['complete_run']
assert parse_interval_velocity_stats(partial)['scope']=='partial_post_ramp_until_stop'
try:parse_interval_velocity_stats(partial[:-1]+['STOP: 3-second test complete'])
except ValueError:pass
else:raise AssertionError('Partial post-ramp coverage accepted as completed run')
assert parse_interval_velocity_stats([]) is None
for bad in (lines[1:],lines+[summary],
            [line.replace('covered_us=998000','covered_us=900000') for line in lines],
            [line.replace('mean_motor_rad_s=6.3','mean_motor_rad_s=6.0') for line in lines],
            [line.replace('rms_error_rad_s=.4','rms_error_rad_s=nan') for line in lines],
            [line.replace('hard_abs_rad_s=8.5','hard_abs_rad_s=9') for line in lines],
            [line.replace('start_us=2000000','start_us=800000') for line in lines]):
    try:parse_interval_velocity_stats(bad)
    except (ValueError,RuntimeError):pass
    else:raise AssertionError('Wrong higher-range protection/statistics accepted')
early=[GUARD_CONFIG,STATS_CONFIG,'VELOCITY_STATS valid=0 invalid=0 samples=0','STOP: reference speed envelope exceeded']
assert parse_interval_velocity_stats(early)['available'] is False
try:verify_speed_guard_config(GUARD_CONFIG.replace('deviation_rad_s=2','deviation_rad_s=3'))
except RuntimeError:pass
else:raise AssertionError('Larger tracking deviation accepted')
print('PASS: higher-range guard limits, full post-ramp2ms statistics, units/displacement/coverage, missing/malformed data and early fault')
