"""Kinematic sizing only. No serial, firmware build, upload or motor commands."""
import argparse,json,math
from pathlib import Path

def size_motion(joint_deg,duration_ms,ratio=8.0,mass_kg=.5,flight_height_m=.2,encoder_counts=4000,pole_pairs=7):
    values=(joint_deg,duration_ms,ratio,mass_kg,flight_height_m,encoder_counts,pole_pairs)
    if any(not math.isfinite(v) or v<=0 for v in values):raise ValueError('All sizing inputs must be finite and positive')
    duration=duration_ms/1000;joint=math.radians(joint_deg);motor=joint*ratio
    mean_rpm=motor/duration*30/math.pi;takeoff=math.sqrt(2*9.80665*flight_height_m)
    profiles={}
    for name,polynomial,peak_factor,accel_factor,end_factor in (
        ('quintic_rest_to_rest','10*u^3 - 15*u^4 + 6*u^5',1.875,10/math.sqrt(3),0),
        ('quartic_accelerating_takeoff','2*u^3 - u^4',2.0,3.0,2.0)):
        # Transition spacing is idealized; physical ABI edges can be uneven.
        peak_rpm=mean_rpm*peak_factor;end_joint_speed=end_factor*joint/duration
        profiles[name]={'normalized_position_polynomial':polynomial,'peak_to_mean_velocity_ratio':peak_factor,
            'peak_motor_rpm':peak_rpm,'peak_motor_rad_s':peak_factor*motor/duration,
            'peak_joint_deg_s':peak_factor*joint_deg/duration,'peak_motor_accel_rad_s2':accel_factor*motor/duration**2,
            'end_joint_rad_s':end_joint_speed,'end_motor_rpm':mean_rpm*end_factor,
            'peak_ABI_edges_per_s':peak_rpm/60*encoder_counts,'minimum_ABI_edge_interval_us':1e6/(peak_rpm/60*encoder_counts),
            'peak_electrical_Hz':peak_rpm/60*pole_pairs,
            'required_vertical_COM_jacobian_at_takeoff_m_per_rad':takeoff/end_joint_speed if end_joint_speed else None}
    return {'scope':'Required kinematics for the supplied travel and duration; not a feasible-drive or jump-performance guarantee.',
        'inputs':{'joint_travel_deg':joint_deg,'duration_ms':duration_ms,'motor_to_joint_ratio':ratio,'mass_kg':mass_kg,
                  'flight_COM_rise_m':flight_height_m,'ABI_counts_per_motor_turn':encoder_counts,'pole_pairs':pole_pairs},
        'motor_travel_deg':joint_deg*ratio,'motor_travel_turns':joint_deg*ratio/360,
        'mean_joint_deg_s':joint_deg/duration,'mean_motor_rpm':mean_rpm,
        'ideal_ballistic_takeoff_speed_m_s':takeoff,'ideal_flight_kinetic_energy_J':mass_kg*9.80665*flight_height_m,
        'profiles':profiles,
        'assumptions':[f'{joint_deg:g} degrees denotes joint travel, not an absolute endpoint.',
            'Trajectory time ends at takeoff for the accelerating candidate; its nonzero endpoint speed requires separate post-takeoff handling.',
            'Flight height means COM rise from takeoff to apex. Air drag and losses are omitted.',
            'COM height Jacobian and actual link geometry are not supplied. A required Jacobian is a consistency condition, not a measured property.',
            'The measured ABI resolution and 7 pole pairs are used only for event-rate/electrical-frequency sizing.',
            'No electrical/current/voltage/position/speed protection setting is changed by this calculation.']}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint-deg',type=float,required=True);parser.add_argument('--duration-ms',type=float,required=True)
    parser.add_argument('--ratio',type=float,default=8);parser.add_argument('--output',type=Path)
    args=parser.parse_args();result=size_motion(args.joint_deg,args.duration_ms,args.ratio)
    if args.output:
        with args.output.open('x',encoding='utf-8') as file:json.dump(result,file,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2,allow_nan=False))

if __name__=='__main__':main()
