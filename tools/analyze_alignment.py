"""Compare commanded fields, ABI angles and nominal current vectors during alignment."""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import re
import statistics


def stats(values):
    return {"min": min(values), "max": max(values), "mean": statistics.mean(values),
            "rms": math.sqrt(statistics.mean(x*x for x in values))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples-csv", type=Path)
    args = parser.parse_args()
    lines = args.log.read_text(encoding="utf-8").splitlines()
    start = next(n for n, line in enumerate(lines) if line.startswith("align_us,"))
    end = lines.index("TRACE_END", start)
    # The host can insert status lines into a dump while sending its final stop.
    data = [lines[start]] + [line for line in lines[start+1:end] if re.match(r"^\d+,", line)]
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO("\n".join(data)))]
    forward = [r for r in rows if r["leg"] == 0]
    reverse = list(reversed([r for r in rows if r["leg"] == 1]))
    metadata = next((line for line in lines if line.startswith("ALIGN_TRACE_CONFIG ")), "")
    configuration = {key: int(value) for key, value in re.findall(r"([a-z_]+)=(\d+)", metadata)}
    per_leg = configuration.get("moving_samples_per_leg", 61)
    expected = configuration.get("expected_samples", 125)
    if expected != 2 * per_leg + 3 or len(forward) != per_leg or len(reverse) != per_leg or len(rows) != expected:
        raise ValueError("Incomplete three-cycle bidirectional alignment survey")
    offsets = [float(v) for v in next(line for line in lines if line.startswith("SENSE_OFFSETS_ADC ")).split(" ",1)[1].split(",")]
    a0 = rows[0]["encoder_angle_rad"]
    direction = 1 if forward[-1]["encoder_angle_rad"] > a0 else -1
    scale = (3.3/4096)/(.003*(-64/7))
    for row in rows:
        a,b,c = [(row[key]-offset)*scale for key,offset in zip(("ia_adc","ib_adc","ic_adc"),offsets)]
        alpha,beta = (2*a-b-c)/3,(b-c)/math.sqrt(3)
        phase = row["field_displacement_rad"]
        current_angle = math.atan2(beta,alpha)
        row["nominal_current_vector_A"] = math.hypot(alpha,beta)
        row["field_current_angle_error_rad"] = math.remainder(current_angle-phase,2*math.pi)
        row["encoder_field_error_electrical_rad"] = direction*7*(row["encoder_angle_rad"]-a0)-phase
        row["encoder_current_error_electrical_rad"] = math.remainder(row["encoder_field_error_electrical_rad"]-row["field_current_angle_error_rad"],2*math.pi)
        row["vbus_nominal_V"] = row["vbus_adc"]*(3.3/4096)*(187/18)
    groups = {}
    for leg,label in ((0,"forward"),(1,"reverse")):
        moving = [r for r in rows if r["leg"]==leg and .3<r["field_displacement_rad"]<18.6]
        groups[label] = {key: stats([r[key] for r in moving]) for key in
                         ("encoder_field_error_electrical_rad","field_current_angle_error_rad","encoder_current_error_electrical_rad","nominal_current_vector_A")}
    pairs=[]
    for a,b in zip(forward,reverse):
        if abs(a["field_displacement_rad"]-b["field_displacement_rad"])>1e-5:
            raise ValueError("Unmatched forward/reverse phase samples")
        if .3<a["field_displacement_rad"]<18.6:
            pairs.append({"field_rad":a["field_displacement_rad"],
                          "midpoint_field_error_rad":(a["encoder_field_error_electrical_rad"]+b["encoder_field_error_electrical_rad"])/2,
                          "hysteresis_rad":b["encoder_field_error_electrical_rad"]-a["encoder_field_error_electrical_rad"]})
    result={"log":args.log.name,"samples":len(rows),"sampling_configuration":configuration,"direction":direction,"by_leg":groups,
            "paired_midpoint_field_error_electrical_rad":stats([r["midpoint_field_error_rad"] for r in pairs]),
            "paired_hysteresis_electrical_rad":stats([r["hysteresis_rad"] for r in pairs]),
            "vbus_nominal_V":stats([r["vbus_nominal_V"] for r in rows]),
            "settled_endpoints":[r for r in rows if r["leg"]>=2],
            "stop_acknowledged":"HOST final_stop_acknowledged=True" in lines,
            "note":"ABI and nominal current observations; field tracking includes drive distortion, rotor lag/cogging and sensor error. No absolute angle calibration inferred."}
    output=json.dumps(result,indent=2)+"\n"
    if args.output: args.output.write_text(output,encoding="utf-8")
    if args.samples_csv:
        with args.samples_csv.open("w",encoding="utf-8",newline="") as dest:
            writer=csv.DictWriter(dest,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(output)


if __name__=="__main__":
    main()
