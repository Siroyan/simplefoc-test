"""Analyze nominal phase currents saved with ABI/PWM trace; no hardware access."""
import argparse
import csv
import io
import itertools
import json
import math
from pathlib import Path
import re


def moments(values):
    return {"min": min(values), "max": max(values), "mean": sum(values) / len(values),
            "rms": math.sqrt(sum(v * v for v in values) / len(values))}


def correlation(a, b):
    ma, mb = sum(a) / len(a), sum(b) / len(b)
    aa = sum((v - ma) ** 2 for v in a)
    bb = sum((v - mb) ** 2 for v in b)
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(aa * bb) if aa * bb else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples-csv", type=Path)
    args = parser.parse_args()
    lines = args.log.read_text(encoding="utf-8").splitlines()
    if any(line.startswith("DRIVE phase_pulse=") for line in lines):
        raise SystemExit("Static phase pulses have no calibrated rotor angle; use phase-pulse analysis")
    offset_line = next((line for line in lines if line.startswith("SENSE_OFFSETS_ADC ")), None)
    if offset_line is None:
        raise SystemExit("Current observer configuration required")
    offsets = list(map(float, offset_line.split(" ", 1)[1].split(",")))
    reference_line = next(line for line in lines if line.startswith("RUN_REFERENCE "))
    reference = {k: float(v) for k, v in re.findall(r"([a-z_]+)=(-?[0-9.]+)", reference_line)}
    start = next(n for n, line in enumerate(lines) if line.startswith("trace_us,"))
    end = lines.index("TRACE_END", start)
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO("\n".join(lines[start:end])))]
    if not rows or not all(row.get("adc1_frames", 0) > 0 for row in rows):
        raise SystemExit("Current observer trace required")
    converted = []
    gain = (3.3 / 4096) / (.003 * (-64 / 7))
    for row in rows:
        currents = [(row[key] - offset) * gain for key, offset in zip(("ia_adc", "ib_adc", "ic_adc"), offsets)]
        a, b, c = currents
        theta = (reference["sensor_direction"] * 7 * row["angle_rad"] - reference["zero_electric_angle_rad"]) % math.tau
        alpha, beta = (2 * a - b - c) / 3, (b - c) / math.sqrt(3)
        converted.append({"trace_us": row["trace_us"], "angle_rad": row["angle_rad"],
                          "velocity_rad_s": row["window10ms_rad_s"] * reference["sensor_direction"],
                          "uq_command_V": row["Uq_V"], "ia_nominal_A": a, "ib_nominal_A": b, "ic_nominal_A": c,
                          "sum_nominal_A": a + b + c,
                          "id_nominal_A": alpha * math.cos(theta) + beta * math.sin(theta),
                          "iq_nominal_A": beta * math.cos(theta) - alpha * math.sin(theta),
                          "vbus_nominal_V": row["vbus_adc"] * (3.3 / 4096) * (187 / 18),
                          "electrical_angle_rad": theta, "adc_age_us": row["adc_age_us"]})
    settled = [row for row in converted if row["trace_us"] >= 800000]
    metrics = {}
    full_metrics = {}
    for key in ("ia_nominal_A", "ib_nominal_A", "ic_nominal_A", "sum_nominal_A", "id_nominal_A", "iq_nominal_A", "vbus_nominal_V"):
        metrics[key] = moments([row[key] for row in settled]) if settled else None
        full_metrics[key] = moments([row[key] for row in converted])
    groups = {}
    for label, predicate in (("slow", lambda v: abs(v) < .2), ("moving", lambda v: v > .2), ("fast", lambda v: v > 2)):
        group = [row for row in settled if predicate(row["velocity_rad_s"])]
        groups[label] = {"samples": len(group), "iq": moments([row["iq_nominal_A"] for row in group]) if group else None,
                         "uq": moments([row["uq_command_V"] for row in group]) if group else None}
    tracking = None
    has_current_control = any(line.startswith(("DRIVE fixed_iq_nominal_A=", "DRIVE current_velocity=")) for line in lines)
    if has_current_control:
        controlled = [row for row in rows if row["trace_us"] >= 400000]
        if controlled:
            errors = [row["iq_filtered_nominal_A"] - row["iq_target_nominal_A"] for row in controlled]
            tracking = {"covered_start_us": controlled[0]["trace_us"], "covered_end_us": controlled[-1]["trace_us"],
                        "filtered_iq_nominal_A": moments([row["iq_filtered_nominal_A"] for row in controlled]),
                        "target_nominal_A": moments([row["iq_target_nominal_A"] for row in controlled]),
                        "filtered_id_nominal_A": moments([row["id_filtered_nominal_A"] for row in controlled]),
                        "filtered_iq_error_nominal_A": moments(errors),
                        "max_voltage_vector_V": max(math.hypot(row["Uq_V"], row["Ud_V"]) for row in rows),
                        "Ud_range_V": [min(row["Ud_V"] for row in rows), max(row["Ud_V"] for row in rows)]}
    permutations = []
    for order in itertools.permutations(("ia_nominal_A", "ib_nominal_A", "ic_nominal_A")):
        d, q = [], []
        for row in settled:
            a, b, c = [row[key] for key in order]
            alpha, beta = (2 * a - b - c) / 3, (b - c) / math.sqrt(3)
            theta = row["electrical_angle_rad"]
            d.append(alpha * math.cos(theta) + beta * math.sin(theta))
            q.append(beta * math.cos(theta) - alpha * math.sin(theta))
        permutations.append({"order": order, "id": moments(d) if d else None, "iq": moments(q) if q else None,
                             "uq_iq_correlation": correlation([row["uq_command_V"] for row in settled], q) if q else None})
    result = {"log": args.log.name, "samples": len(rows), "offsets_adc": offsets,
              "voltage_vector_max_V": max(math.hypot(row["Uq_V"], row.get("Ud_V", 0)) for row in rows),
              "current_control_error_full_trace": moments([row["iq_filtered_nominal_A"] - row["iq_target_nominal_A"] for row in rows]) if has_current_control else None,
              "last_current_control_sample": {key: rows[-1][key] for key in ("trace_us", "iq_target_nominal_A", "iq_filtered_nominal_A", "id_filtered_nominal_A", "Uq_V", "Ud_V")} if has_current_control else None,
              "max_adc_age_us": max(row["adc_age_us"] for row in rows),
              "frame_difference_range": [min(row["adc2_frames"] - row["adc1_frames"] for row in rows),
                                          max(row["adc2_frames"] - row["adc1_frames"] for row in rows)],
              "metrics_after_0_8s": metrics if settled else None, "metrics_full_trace": full_metrics,
              "temperature_adc_range": [min(row["temp_adc"] for row in rows), max(row["temp_adc"] for row in rows)],
              "groups_after_0_8s": groups,
              "current_tracking_after_0_4s": tracking,
              "phase_permutation_diagnostics": permutations,
              "note": "Nominal scaling, sample-based statistics; permutation fits are hypotheses, not phase/sign calibration. Uq is a command. VBUS is not a CV flag."}
    output = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    if args.samples_csv:
        with args.samples_csv.open("w", encoding="utf-8", newline="") as destination:
            writer = csv.DictWriter(destination, fieldnames=list(converted[0]))
            writer.writeheader()
            writer.writerows(converted)
    print(json.dumps({k: v for k, v in result.items() if k != "phase_permutation_diagnostics"}, indent=2))


if __name__ == "__main__":
    main()
