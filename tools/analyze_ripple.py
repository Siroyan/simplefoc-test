"""Exploratory weighted sinusoidal fits of 10ms ABI speed ripple; no hardware access."""
import argparse
from bisect import bisect_right
import csv
import io
import json
import math
from pathlib import Path
import re


def solve_weighted(basis, values, weights):
    size=len(basis[0])
    matrix = [[sum(w*x[i]*x[j] for w,x in zip(weights,basis)) for j in range(size)] +
              [sum(w*x[i]*v for w,x,v in zip(weights,basis,values))] for i in range(size)]
    for i in range(size):
        pivot = max(range(i,size),key=lambda j:abs(matrix[j][i]))
        matrix[i],matrix[pivot] = matrix[pivot],matrix[i]
        divisor = matrix[i][i]
        if abs(divisor)<1e-12: return None
        matrix[i] = [v/divisor for v in matrix[i]]
        for j in range(size):
            if j!=i:
                scale=matrix[j][i]
                matrix[j]=[a-scale*b for a,b in zip(matrix[j],matrix[i])]
    return [matrix[i][size] for i in range(size)]


def explained_variance(basis,values,weights,coefficients):
    mean=sum(w*v for w,v in zip(weights,values))/sum(weights)
    total=sum(w*(v-mean)**2 for w,v in zip(weights,values))
    error=sum(w*(v-sum(a*b for a,b in zip(coefficients,x)))**2 for w,v,x in zip(weights,values,basis))
    return 1-error/total if total else 0


def fit_sinusoid(phases, values, weights):
    basis = [[1, math.cos(p), math.sin(p)] for p in phases]
    coefficients=solve_weighted(basis,values,weights)
    if coefficients is None: return None
    return {"explained_variance":explained_variance(basis,values,weights,coefficients),
            "amplitude_rad_s":math.hypot(*coefficients[1:]),
            "phase_rad":math.atan2(coefficients[2],coefficients[1]),
            "fitted_error_offset_rad_s":coefficients[0]}


def holdout_fit(angles,values,weights,orders,cut):
    basis=[[1]+[v for order in orders for v in (math.cos(order*a),math.sin(order*a))] for a in angles]
    coefficients=solve_weighted(basis[:cut],values[:cut],weights[:cut])
    if coefficients is None: return None
    return {"orders":orders,"train_samples":cut,"test_samples":len(values)-cut,
            "train_explained_variance":explained_variance(basis[:cut],values[:cut],weights[:cut],coefficients),
            "test_explained_variance":explained_variance(basis[cut:],values[cut:],weights[cut:],coefficients),
            "training_coefficients":[{"order":order,"cos":coefficients[1+2*n],"sin":coefficients[2+2*n]}
                                     for n,order in enumerate(orders)]}


def window_midpoint_us(row):
    if row.get("window_age_us",0)>=65535:
        raise ValueError("Saturated window age cannot support a phase fit")
    return row["trace_us"]-row.get("window_age_us",0)-5000


def window_midpoint_angle(row, sample_times_us, sample_angles):
    if "window_age_us" not in row:
        return row["angle_rad"]-row["window10ms_rad_s"]*.005
    center=window_midpoint_us(row)
    n=bisect_right(sample_times_us,center)-1
    if n<0 or n+1>=len(sample_times_us): return math.nan
    fraction=(center-sample_times_us[n])/(sample_times_us[n+1]-sample_times_us[n])
    return sample_angles[n]+fraction*(sample_angles[n+1]-sample_angles[n])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    lines=args.log.read_text(encoding="utf-8").splitlines()
    first=next(n for n,line in enumerate(lines) if line.startswith("trace_us,"))
    last=lines.index("TRACE_END",first)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO("\n".join(lines[first:last])))]
    reference=next(line for line in lines if line.startswith("RUN_REFERENCE "))
    command_direction=-1 if sum(r["target_rad_s"] for r in rows if r["trace_us"]>=800000)<0 else 1
    direction=float(re.search(r"sensor_direction=(-?[0-9.]+)",reference)[1])
    start_angle=float(re.search(r"start_angle_rad=(-?[0-9.]+)",reference)[1])
    index_angles=[float(re.search(r"encoder_angle_rad=(-?[0-9.]+)",line)[1]) for line in lines
                  if line.startswith("INDEX_REFERENCE ") and "encoder_angle_rad=" in line]
    index_consistent=all(abs((a-index_angles[0])/(2*math.pi)-round((a-index_angles[0])/(2*math.pi)))<=2/4000
                         for a in index_angles) if index_angles else None
    if index_angles and index_consistent:
        start_angle=index_angles[0]
    zero=float(re.search(r"zero_electric_angle_rad=(-?[0-9.]+)",reference)[1])
    sample_times=[r["trace_us"] for r in rows]; sample_angles=[r["angle_rad"] for r in rows]
    has_window_age="window_age_us" in rows[0]
    rows=[r for r in rows if r["trace_us"]>=800000]
    values=[];angles=[];times=[];weights=[];angle_steps=[]
    for a,b in zip(rows,rows[1:]):
        weight=(b["trace_us"]-a["trace_us"])/1e6
        if weight<=0: continue
        center_angle=window_midpoint_angle(a,sample_times,sample_angles)
        next_center_angle=window_midpoint_angle(b,sample_times,sample_angles)
        if not math.isfinite(center_angle) or not math.isfinite(next_center_angle): continue
        velocity=command_direction*direction*a["window10ms_rad_s"]
        values.append(velocity-1)
        angles.append(direction*(center_angle-start_angle))
        times.append(window_midpoint_us(a)/1e6)
        weights.append(weight)
        angle_steps.append(abs(next_center_angle-center_angle))
    if len(values)<30: raise ValueError("Need at least 30 covered samples after .8s")
    spatial=[];temporal=[]
    for order in range(1,101):
        fit=fit_sinusoid([order*a for a in angles],values,weights)
        if fit:
            fit.update({"cycles_per_mechanical_turn":order,
                        "interval_fraction_over_half_cycle":sum(w for w,a in zip(weights,angle_steps) if order*a>math.pi)/sum(weights)})
            spatial.append(fit)
    for half_hz in range(2,81):
        freq=half_hz/2
        fit=fit_sinusoid([2*math.pi*freq*t for t in times],values,weights)
        if fit: fit["frequency_Hz"]=freq; temporal.append(fit)
    result={"analysis_format_version":2,"log":args.log.name,"samples":len(values),"covered_seconds":sum(weights),
            "command_direction":command_direction,
            "angle_anchor":"Index rising edge (+/-1 ABI count uncertainty)" if index_angles and index_consistent else "alignment endpoint; cross-boot phase is unverified",
            "index_angles_rad":index_angles,"index_turn_spacing_within_2_counts":index_consistent,
            "zero_electrical_from_index_rad":(zero-direction*7*index_angles[0])%(2*math.pi) if index_angles and index_consistent else None,
            "fit_formula":"speed_error = offset + amplitude*cos(phase_argument - phase_rad)",
            "window_timing":"recorded window age; nominal 10ms span; angle interpolated at window midpoint" if has_window_age else "unrecorded window age; approximate midpoint; cross-run phase is uncertain",
            "temporal_time_basis":"velocity window midpoint: trace_us - window_age_us - 5000" if has_window_age else "approximate velocity window midpoint: trace_us - 5000; window age unavailable",
            "abi_quality_metadata":[line for line in lines if line.startswith("ABI_QUALITY")],
            "within_run_holdout":[holdout_fit(angles,values,weights,orders,len(values)//2) for orders in
                                  ([],[84],[14,84],[14,28,84])],
            "selected_spatial_orders":[r for r in spatial if r["cycles_per_mechanical_turn"] in (7,14,21,42,84)],
            "spatial_top8":sorted(spatial,key=lambda r:r["explained_variance"],reverse=True)[:8],
            "temporal_top8":sorted(temporal,key=lambda r:r["explained_variance"],reverse=True)[:8],
            "temporal_band_peaks":[{"band_Hz":[low,high],**max((r for r in temporal if low<=r['frequency_Hz']<=high),key=lambda r:r['explained_variance'])} for low,high in ((3,8),(8,20),(20,40))],
            "note":"Exploratory single-frequency fits, no multiple-search significance claim. Angle is approximate 10ms window midpoint; high orders can alias. A harmonic match alone does not establish cogging, drive distortion, sensor error or control oscillation."}
    output=json.dumps(result,indent=2)+"\n"
    if args.output: args.output.write_text(output,encoding="utf-8")
    print(output)


if __name__=="__main__":
    main()
