"""Exploratory current/acceleration/position regression; never exports firmware calibration."""
import argparse
from bisect import bisect_right
import csv
import io
import json
import math
from pathlib import Path
import re
from analyze_ripple import solve_weighted,explained_variance


def local_motion(rows,center):
    origin=rows[center]["trace_us"]
    subset=rows[center-2:center+3]
    times=[(r["trace_us"]-origin)/1e6 for r in subset]
    coefficients=solve_weighted([[1,t,t*t] for t in times],[r["angle_rad"] for r in subset],[1]*5)
    return coefficients[1],2*coefficients[2]


def interpolate(times,values,time):
    at=bisect_right(times,time)-1
    if at<0 or at+1>=len(times): return math.nan
    portion=(time-times[at])/(times[at+1]-times[at])
    return values[at]+portion*(values[at+1]-values[at])


def prepare(path,orders,lead_us):
    lines=path.read_text(encoding="utf-8").splitlines()
    first=next(n for n,l in enumerate(lines) if l.startswith("trace_us,"));end=lines.index("TRACE_END",first)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO("\n".join(lines[first:end])))]
    ref=next(l for l in lines if l.startswith("RUN_REFERENCE "))
    direction=float(re.search(r"sensor_direction=(-?[0-9.]+)",ref)[1])
    index=next((l for l in lines if l.startswith("INDEX_REFERENCE ") and "encoder_angle_rad=" in l),None)
    anchor=float(re.search(r"encoder_angle_rad=(-?[0-9.]+)",index)[1]) if index else float(re.search(r"start_angle_rad=(-?[0-9.]+)",ref)[1])
    times=[r["trace_us"] for r in rows];currents=[r["iq_filtered_nominal_A"] for r in rows]
    features=[];targets=[];coordinates=[]
    for n in range(2,len(rows)-2):
        row=rows[n]
        if row["trace_us"]<800000: continue
        velocity,acceleration=local_motion(rows,n)
        velocity*=direction;acceleration*=direction
        if velocity<=.2: continue # No linear Coulomb/viscous model for sticking.
        iq=interpolate(times,currents,row["trace_us"]+lead_us)
        if not math.isfinite(iq): continue
        angle=direction*(row["angle_rad"]-anchor)
        features.append([1,iq,velocity]+[x for order in orders for x in (math.cos(order*angle),math.sin(order*angle))])
        targets.append(acceleration);coordinates.append(angle)
    if len(features)<50: raise ValueError("Insufficient moving samples")
    return features,targets,index is not None


def describe(coefficients,orders):
    gain=coefficients[1]
    return {"acceleration_per_nominal_A":gain,"constant_acceleration":coefficients[0],
            "velocity_coefficient":coefficients[2],
            "equivalent_current_terms":[{"order":order,"cos_A":-coefficients[3+2*n]/gain,
                                          "sin_A":-coefficients[4+2*n]/gain} for n,order in enumerate(orders)] if gain>0 else None}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log",type=Path)
    parser.add_argument("--compare",type=Path,nargs="*",default=[])
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    results=[]
    for orders in ([],[84],[14,28,84]):
        for lead_us in (0,2000,4000):
            basis,values,has_index=prepare(args.log,orders,lead_us)
            cut=len(values)//2
            train=solve_weighted(basis[:cut],values[:cut],[1]*cut)
            whole=solve_weighted(basis,values,[1]*len(values))
            if train is None or whole is None: continue
            tests=[]
            for other in args.compare:
                x,y,index=prepare(other,orders,lead_us)
                tests.append({"log":other.name,"has_index":index,
                              "whole_reference_model_R2":explained_variance(x,y,[1]*len(y),whole)})
            results.append({"orders":orders,"iq_lead_us":lead_us,"samples":len(values),"has_index":has_index,
                            "whole_fit_R2":explained_variance(basis,values,[1]*len(values),whole),
                            "within_run_test_R2":explained_variance(basis[cut:],values[cut:],[1]*(len(values)-cut),train),
                            **describe(whole,orders),"other_logs":tests})
    result={"log":args.log.name,"models":results,
            "note":"Exploratory least squares, local quadratic acceleration over five 10ms samples; nominal current and approximate LPF compensation. Noise, sticking, sampling, sensor nonlinearity, phase calibration and closed-loop correlation bias identification. Negative current gain is nonphysical. No coefficients are approved for drive feedforward."}
    output=json.dumps(result,indent=2)+"\n"
    if args.output: args.output.write_text(output,encoding="utf-8")
    print(output)


if __name__=="__main__":
    main()
