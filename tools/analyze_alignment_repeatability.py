"""Group Index-referenced alignment endpoints by motor pole; never exports calibration."""
import argparse
import cmath
import glob
import json
import math
from pathlib import Path
import re
import statistics
from analyze_ripple import solve_weighted


def values(line):
    return {key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=(-?[0-9.]+)",line)}


def endpoint(path):
    lines=path.read_text(encoding="utf-8").splitlines()
    if "HOST final_stop_acknowledged=True" not in lines:
        return None
    pwm=next((values(line) for line in lines if line.startswith("PWM ")),{})
    ref=next((values(line) for line in lines if line.startswith("RUN_REFERENCE ")),None)
    indices=[values(line)["encoder_angle_rad"] for line in lines if line.startswith("INDEX_REFERENCE ") and "encoder_angle_rad=" in line]
    if pwm.get("hz")!=12500 or not ref or not indices:
        return None
    anchor=indices[0]
    unwrapped=[anchor+math.remainder(angle-anchor,math.tau) for angle in indices]
    if max(unwrapped)-min(unwrapped)>2*math.tau/4000+1e-5:
        raise ValueError(f"Inconsistent Index observations in {path}")
    anchor=statistics.median(unwrapped)
    direction=ref["sensor_direction"]
    if direction not in (-1,1):
        raise ValueError("Known encoder direction required")
    period=math.tau/7
    start=ref["start_angle_rad"]
    zero=ref["zero_electric_angle_rad"]
    # The offset captured at the alignment endpoint loses the pole number.
    # Recover its branch using the nearby run start; relaxation is separately
    # recorded rather than treating the run-start angle as the held endpoint.
    held=start+math.remainder(direction*zero/7-start,period)
    angle=(direction*(held-anchor))%math.tau
    profile=next((values(line) for line in lines if line.startswith("ALIGN_PROFILE ")), {})
    return {"log":path.name,"alignment_cycles":int(profile.get("cycles",3)),"held_angle_from_index_rad":angle,
            "zero_electrical_from_index_rad":(zero-direction*7*anchor)%math.tau,
            "run_start_minus_held_rad":direction*(start-held),"index_observations":len(indices)}


def analyze(observations):
    if len(observations)<3:
        raise ValueError("Need at least three Index-referenced completed runs")
    mean_phase=cmath.phase(sum(cmath.exp(1j*o["zero_electrical_from_index_rad"]) for o in observations))%math.tau
    base=mean_phase/7
    period=math.tau/7
    for row in observations:
        row["pole_group"]=round((row["held_angle_from_index_rad"]-base)/period)%7
        expected=base+row["pole_group"]*period
        row["mechanical_residual_rad"]=math.remainder(row["held_angle_from_index_rad"]-expected,math.tau)
    groups=[]
    for pole in range(7):
        selected=[o for o in observations if o["pole_group"]==pole]
        if selected:
            residuals=[o["mechanical_residual_rad"] for o in selected]
            groups.append({"pole_group":pole,"runs":len(selected),
                           "mean_residual_rad":statistics.mean(residuals),
                           "within_group_std_rad":statistics.pstdev(residuals),
                           "residual_range_rad":[min(residuals),max(residuals)]})
    shape_models=[]
    for orders in ([1],[1,2]):
        parameters=1+2*len(orders)
        if len(groups)<=parameters:
            continue
        basis=[[1]+[v for n in orders for v in (math.cos(n*(base+g["pole_group"]*period)),math.sin(n*(base+g["pole_group"]*period)))] for g in groups]
        residuals=[g["mean_residual_rad"] for g in groups]
        fitted=solve_weighted(basis,residuals,[1]*len(groups))
        if fitted is None:
            continue
        folds=[]
        for held in range(len(groups)):
            train=[n for n in range(len(groups)) if n!=held]
            coefficients=solve_weighted([basis[n] for n in train],[residuals[n] for n in train],[1]*len(train))
            if coefficients is None:break
            predicted=sum(a*b for a,b in zip(coefficients,basis[held]))
            folds.append({"pole_group":groups[held]["pole_group"],"prediction_rad":predicted,
                          "observed_rad":residuals[held],"error_rad":predicted-residuals[held],
                          "constant_error_rad":statistics.mean(residuals[n] for n in train)-residuals[held]})
        shape_models.append({"mechanical_orders":orders,"pole_count":len(groups),"mean_offset_rad":fitted[0],
                             "terms":[{"order":n,"cos_rad":fitted[1+2*i],"sin_rad":fitted[2+2*i],
                                       "amplitude_rad":math.hypot(fitted[1+2*i],fitted[2+2*i])} for i,n in enumerate(orders)],
                             "leave_one_pole_out_error_rms_rad":math.sqrt(statistics.mean(f["error_rad"]**2 for f in folds)) if len(folds)==len(groups) else None,
                             "constant_leave_one_pole_out_error_rms_rad":math.sqrt(statistics.mean(f["constant_error_rad"]**2 for f in folds)) if len(folds)==len(groups) else None,
                             "folds":folds})
    return {"runs":len(observations),"reference_mean_phase_rad":mean_phase,
            "observed_pole_groups":len(groups),"missing_pole_groups":[p for p in range(7) if p not in {g["pole_group"] for g in groups}],
            "groups":groups,"observations":observations,"exploratory_shape_models":shape_models,
            "note":"Pole groups assume the .7V alignment endpoint selects one of seven nearby stable field orientations; the branch is chosen near the run start. Residuals mix encoder nonlinearity, pole placement, friction/load and alignment repeatability. Index visibility can bias the sampled poles. No absolute angle calibration or firmware correction is inferred."}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("patterns",nargs="+")
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    paths=sorted({Path(name) for pattern in args.patterns for name in glob.glob(pattern)})
    observations=[row for path in paths if (row:=endpoint(path)) is not None]
    output=json.dumps(analyze(observations),indent=2)+"\n"
    if args.output:args.output.write_text(output,encoding="utf-8")
    print(output)


if __name__=="__main__":
    main()
