"""Estimate relative current-channel scales using three-wire current balance, without applying calibration."""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import re
import statistics
from analyze_ripple import solve_weighted


def metrics(values):
    return {"mean_A":statistics.mean(values),"rms_A":math.sqrt(statistics.mean(x*x for x in values)),
            "std_A":statistics.pstdev(values)}


def load_vectors(path):
    lines=path.read_text(encoding="utf-8").splitlines()
    offsets=[float(x) for x in next(l for l in lines if l.startswith("SENSE_OFFSETS_ADC ")).split(" ",1)[1].split(",")]
    start=next(n for n,l in enumerate(lines) if l.startswith(("trace_us,","align_us,")))
    end=lines.index("TRACE_END",start)
    table=[lines[start]]+[l for l in lines[start+1:end] if re.match(r"^\d+,",l)]
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO("\n".join(table)))]
    scale=(3.3/4096)/(.003*(-64/7))
    vectors=[tuple((row[key]-zero)*scale for key,zero in zip(("ia_adc","ib_adc","ic_adc"),offsets)) for row in rows]
    return rows,vectors


def fit_relative(vectors):
    coefficients=solve_weighted([[1,a,b] for a,b,c in vectors],[-c for a,b,c in vectors],[1]*len(vectors))
    if coefficients is None: raise ValueError("Reference does not span independent phase-current directions")
    unscaled=[coefficients[1],coefficients[2],1]
    mean=statistics.mean(unscaled)
    if mean<=0: raise ValueError("Nonphysical fitted scales")
    return {"factors_mean_normalized":[x/mean for x in unscaled],"fitted_sum_offset_A":coefficients[0]/mean}


def evaluate(vectors,factors):
    raw=[sum(v) for v in vectors]
    corrected=[sum(a*b for a,b in zip(v,factors)) for v in vectors]
    return {"samples":len(vectors),"unadjusted_sum":metrics(raw),"gain_only_sum":metrics(corrected)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference",type=Path)
    parser.add_argument("--compare",nargs="*",type=Path,default=[])
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    rows,vectors=load_vectors(args.reference)
    fit=fit_relative(vectors)
    partitions=[]
    if "leg" in rows[0]:
        for leg in (0,1):
            train=[v for r,v in zip(rows,vectors) if r["leg"]==leg]
            test=[v for r,v in zip(rows,vectors) if r["leg"]==1-leg]
            part=fit_relative(train)
            partitions.append({"training_leg":leg,**part,"other_leg":evaluate(test,part["factors_mean_normalized"])})
    result={"reference":args.reference.name,**fit,"reference_balance":evaluate(vectors,fit["factors_mean_normalized"]),
            "direction_holdouts":partitions,"other_logs":[{"log":p.name,**evaluate(load_vectors(p)[1],fit["factors_mean_normalized"])} for p in args.compare],
            "note":"Exploratory relative scales, mean fixed to one; no absolute current calibration. ADC sampling skew, common-mode shunt paths and offset drift can bias current-sum fitting. No firmware changes are made."}
    output=json.dumps(result,indent=2)+"\n"
    if args.output: args.output.write_text(output,encoding="utf-8")
    print(output)


if __name__=="__main__":
    main()
