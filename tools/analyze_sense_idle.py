"""Summarize the gates-OFF ADC capture; currents and VBUS use nominal scaling."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    lines = args.log.read_text(encoding="utf-8").splitlines()
    rows = [list(map(float, line.split(",")[1:])) for line in lines if line.startswith("SENSE,")]
    if len(rows) < 2:
        raise SystemExit("At least two ADC observation rows required")
    span_ms = rows[-1][0] - rows[0][0]
    result = {
        "log": args.log.name,
        "rows": len(rows),
        "adc1_frames_per_s": (rows[-1][1] - rows[0][1]) * 1000 / span_ms,
        "adc2_frames_per_s": (rows[-1][2] - rows[0][2]) * 1000 / span_ms,
        "max_sample_age_us": max(row[3] for row in rows),
        "adc2_minus_adc1_frame_range": [min(row[2] - row[1] for row in rows), max(row[2] - row[1] for row in rows)],
        "all_gates_off": all(row[13] == 1 for row in rows),
        "zero_current_nominal_A": [
            {"mean": sum(row[n] for row in rows) / len(rows), "min": min(row[n] for row in rows),
             "max": max(row[n] for row in rows)} for n in (9, 10, 11)],
        "vbus_nominal_range_V": [min(row[12] for row in rows), max(row[12] for row in rows)],
        "temperature_adc_range": [min(row[8] for row in rows), max(row[8] for row in rows)],
        "offsets": next((line for line in lines if line.startswith("SENSE_OFFSETS_ADC ")), None),
        "initialized_gates_off": "SENSE_INIT ready=1 gates_off=1" in lines,
        "stop_acknowledged": any(line.startswith("HOST final_stop_acknowledged=True") for line in lines),
        "note": "Nominal 3.3V ADC reference and resistor/gain values; phase mapping/sign and absolute accuracy not validated under drive. VBUS is not a CV-mode flag.",
    }
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
