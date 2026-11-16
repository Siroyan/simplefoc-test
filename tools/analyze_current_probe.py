"""Estimate coherent current response to the known small sine probe; no calibration export."""
import argparse
import cmath
import csv
import io
import json
import math
from pathlib import Path
import re
import statistics
from analyze_ripple import solve_weighted,window_midpoint_us


def fit(rows, key, frequency, times_us=None):
    times = [t*1e-6 for t in times_us] if times_us is not None else [r["trace_us"]*1e-6 for r in rows]
    origin = statistics.mean(times)
    basis = [[1, t-origin, math.cos(math.tau*frequency*t), math.sin(math.tau*frequency*t)] for t in times]
    values = [r[key] for r in rows]
    coefficients = solve_weighted(basis, values, [1]*len(rows))
    if coefficients is None:
        raise ValueError("Degenerate probe fit")
    residual = [v-sum(a*b for a,b in zip(x, coefficients)) for x,v in zip(basis, values)]
    drift=solve_weighted([[1,t-origin] for t in times],values,[1]*len(rows))
    drift_error=sum((v-drift[0]-drift[1]*(t-origin))**2 for t,v in zip(times,values))
    return complex(coefficients[2], -coefficients[3]), {
        "amplitude": math.hypot(coefficients[2], coefficients[3]),
        "phase_deg": math.degrees(math.atan2(-coefficients[3],coefficients[2])),
            "residual_rms": math.sqrt(statistics.mean(v*v for v in residual)),
            "sine_R2_after_removing_linear_trend":1-sum(v*v for v in residual)/drift_error if drift_error else 0}


def velocity_response(rows,frequency,amplitude,phase,encoder_direction=1):
    if amplitude<=0:raise ValueError('A nonzero known probe is required')
    if encoder_direction not in (-1,1):raise ValueError('Known encoder direction required')
    stimulus=amplitude*complex(math.sin(phase),-math.cos(phase))
    result={}
    for key in ('filtered_rad_s','window10ms_rad_s'):
        times=[window_midpoint_us(r) for r in rows] if key=='window10ms_rad_s' else None
        corrected=[{**r,key:encoder_direction*r[key]} for r in rows] if key=='window10ms_rad_s' else rows
        component,description=fit(corrected,key,frequency,times)
        ratio=component/stimulus
        result[key]={'gain_rad_s_per_nominal_A':abs(ratio),'phase_deg':math.degrees(cmath.phase(ratio)),
                     'coherent_velocity_component':description,
                     'time_basis':'velocity window midpoint' if times is not None else 'trace timestamp; current controller LPF velocity'}
    return result


def transfer(rows, frequency):
    fitted = {key: fit(rows,key,frequency) for key in ("iq_target_nominal_A","iq_filtered_nominal_A","Uq_V")}
    target = fitted["iq_target_nominal_A"][0]
    response = fitted["iq_filtered_nominal_A"][0]
    valid = abs(target) >= .005
    ratio = response/target if valid else None
    return {"samples":len(rows),"start_us":rows[0]["trace_us"],"end_us":rows[-1]["trace_us"],
            "coherent_components":{key:value[1] for key,value in fitted.items()},
            "target_amplitude_at_least_5mA":valid,
            "current_gain":abs(ratio) if ratio is not None else None,
            "current_phase_deg":math.degrees(cmath.phase(ratio)) if ratio is not None else None,
            "single_frequency_phase_delay_ms":-cmath.phase(ratio)/(math.tau*frequency)*1000 if ratio is not None else None}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    lines=args.log.read_text(encoding="utf-8").splitlines()
    metadata=next(line for line in lines if line.startswith("CURRENT_PROBE "))
    config={k:float(v) for k,v in re.findall(r"([A-Za-z_]+)=([0-9.]+)",metadata)}
    frequency=config["frequency_Hz"]
    reference=next((line for line in lines if line.startswith('RUN_REFERENCE ')),'')
    encoder_direction=float(re.search(r'sensor_direction=(-?[0-9.]+)',reference)[1]) if reference else 1
    start=next(n for n,line in enumerate(lines) if line.startswith("trace_us,"))
    end=lines.index("TRACE_END",start)
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO("\n".join(lines[start:end])))]
    settled_start=config["start_us"]+config["blend_us"]+100000
    settled=[row for row in rows if row["trace_us"]>=settled_start]
    if len(settled)<50:
        raise ValueError("Insufficient settled probe duration")
    summary=next(line for line in lines if line.startswith("CURRENT_PROBE_SUMMARY "))
    if "invalid=0" not in summary:
        raise ValueError("Probe faulted")
    sample_header=next(n for n,line in enumerate(lines) if line.startswith("CURRENT_PROBE_SAMPLES "))
    checked=0;max_wave_error=0;max_total_error=0
    for line in lines[sample_header+1:]:
        parts=line.split(",")
        if len(parts)!=4 or not parts[0].isdigit():
            continue
        elapsed,pid,probe,total=map(float,parts)
        blend=max(0,min(1,(elapsed-config["start_us"])/config["blend_us"]))
        expected_probe=config["amplitude_nominal_A"]*blend*math.sin(math.tau*frequency*elapsed*1e-6+config.get('phase_rad',0))
        expected_total=max(-config["total_bound_A"],min(config["total_bound_A"],pid+probe))
        max_wave_error=max(max_wave_error,abs(probe-expected_probe));max_total_error=max(max_total_error,abs(total-expected_total))
        checked+=1
    if not checked or max_wave_error>2e-6 or max_total_error>2e-6:
        raise ValueError("Logged probe does not match the fitted excitation")
    intervals=[(b["trace_us"]-a["trace_us"])*1e-6 for a,b in zip(settled,settled[1:])]
    if frequency*2*max(intervals)>=1:
        raise ValueError("Sample spacing cannot resolve the probe frequency")
    result={"log":args.log.name,"probe":metadata,"summary":summary,
            "firmware":next((line for line in lines if line.startswith("FIRMWARE ")),None),
            "current_loop":next((line for line in lines if line.startswith("CURRENT_LOOP ")),None),
            "current_active":next((line for line in lines if line.startswith("CURRENT_ACTIVE ")),None),
            "stationary_attempt":bool(config.get("stationary_attempt",0)),
            "settled_angle_span_rad":max(r["angle_rad"] for r in settled)-min(r["angle_rad"] for r in settled),
            "logged_probe_samples_checked":checked,"max_logged_wave_error_nominal_A":max_wave_error,
            "full_settled_fit":transfer(settled,frequency),
            "split_fits":[transfer(part,frequency) for part in (settled[:len(settled)//2],settled[len(settled)//2:])],
            "closed_loop_velocity_response":velocity_response(settled,frequency,config['amplitude_nominal_A'],config.get('phase_rad',0),encoder_direction) if any(l.startswith('DRIVE current_velocity=1 ') for l in lines) else None,
            "split_velocity_responses":[velocity_response(part,frequency,config['amplitude_nominal_A'],config.get('phase_rad',0),encoder_direction) for part in (settled[:len(settled)//2],settled[len(settled)//2:])] if any(l.startswith('DRIVE current_velocity=1 ') for l in lines) else None,
            "excitation_unclipped":int(re.search(r'clipped_calls=(\d+)',summary)[1])==0,
            "stop_acknowledged":"HOST final_stop_acknowledged=True" in lines,
            "note":"Single-frequency coherent fit with offset and linear trend; rotor motion/disturbances can bias the estimate. Velocity ratios use the known added probe, not total PID current, and include the closed velocity loop. Clipping, split fits and opposite-phase repeats must be checked. These are not independently identified open-loop motor parameters, calibrated current, or a full delay measurement."}
    output=json.dumps(result,indent=2)+"\n"
    if args.output:args.output.write_text(output,encoding="utf-8")
    print(output)


if __name__=="__main__":
    main()
