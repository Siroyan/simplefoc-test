"""Validate the opt-in current-PI back-calculation timing experiment."""
CONFIG='CURRENT_TRACKING q_backcalc_s=0.0020 d_backcalc_s=0.0000'
ACTIVE='CURRENT_TRACKING_ACTIVE q_backcalc_s=0.0020 d_backcalc_s=0.0000'

def verify_current_tracking(line,active=False,q_time=.002):
    if q_time not in (.002,.005) or line!=(ACTIVE if active else CONFIG).replace('0.0020',f'{q_time:.4f}'):
        raise RuntimeError('Current-PI tracking time differs from the requested experiment; stopping')

def parse_current_tracking(lines):
    configs=[l for l in lines if l.startswith('CURRENT_TRACKING ')]
    active=[l for l in lines if l.startswith('CURRENT_TRACKING_ACTIVE ')]
    if not configs and not active:return None
    if len(configs)!=1 or len(active)!=1:raise ValueError('Current tracking config and active values required')
    q_time=.005 if configs[0]==CONFIG.replace('0.0020','0.0050') else .002
    verify_current_tracking(configs[0],q_time=q_time);verify_current_tracking(active[0],True,q_time)
    return {'configuration':configs[0],'active':active[0]}
