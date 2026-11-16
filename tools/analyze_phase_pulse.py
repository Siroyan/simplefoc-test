"""Check observed current sign/order against six small static voltage vectors."""
import argparse
import csv
import io
import json
from pathlib import Path


def mean(values):
    return sum(values) / len(values) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    lines = args.log.read_text(encoding="utf-8").splitlines()
    if not any(line.startswith("DRIVE phase_pulse=1") for line in lines):
        raise SystemExit("Expected static phase pulse capture")
    offsets = list(map(float, next(line for line in lines if line.startswith("SENSE_OFFSETS_ADC ")).split(" ", 1)[1].split(",")))
    start = next(n for n, line in enumerate(lines) if line.startswith("trace_us,"))
    end = lines.index("TRACE_END", start)
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO("\n".join(lines[start:end])))]
    gain = (3.3 / 4096) / (.003 * (-64 / 7))
    for row in rows:
        row["currents"] = [(row[key] - offset) * gain for key, offset in zip(("ia_adc", "ib_adc", "ic_adc"), offsets)]
    pulses = []
    for pulse in range(6):
        hold = [row for row in rows if pulse * 250000 + 60000 <= row["trace_us"] < pulse * 250000 + 140000]
        rest = [row for row in rows if pulse * 250000 + 190000 <= row["trace_us"] < (pulse + 1) * 250000]
        if not hold or not rest:
            pulses.append({"pulse": pulse, "complete": False})
            continue
        on = [mean([row["currents"][n] for row in hold]) for n in range(3)]
        off = [mean([row["currents"][n] for row in rest]) for n in range(3)]
        delta = [a - b for a, b in zip(on, off)]
        sign = 1 if pulse % 2 == 0 else -1
        pulses.append({"pulse": pulse, "excited_phase": "UVW"[pulse // 2], "polarity": sign,
                       "complete": True, "hold_samples": len(hold), "rest_samples": len(rest),
                       "mean_on_nominal_A": on, "mean_off_nominal_A": off,
                       "delta_nominal_A": delta, "delta_sum_nominal_A": sum(delta),
                       "expected_signed_delta_nominal_A": delta[pulse // 2] * sign,
                       "largest_abs_response_phase": "UVW"[max(range(3), key=lambda n: abs(delta[n]))]})
    result = {"log": args.log.name, "pulses": pulses,
              "max_adc_age_us": max(row["adc_age_us"] for row in rows),
              "vbus_nominal_range_V": [min(row["vbus_adc"] for row in rows) * (3.3 / 4096) * (187 / 18),
                                        max(row["vbus_adc"] for row in rows) * (3.3 / 4096) * (187 / 18)],
              "angle_span_rad": max(row["angle_rad"] for row in rows) - min(row["angle_rad"] for row in rows),
              "note": "Nominal scaling and on-minus-rest current responses; checks phase/sign, not absolute current calibration or rotor electrical zero."}
    output = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")


if __name__ == "__main__":
    main()
