"""Bin saved ABI observations by calibrated electrical angle; no hardware access.

These are short, angle/time-correlated runs. Bins describe observations and do
not identify whether a pattern comes from cogging, commutation, or feedback.
"""
import argparse
import csv
import io
import math
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    with args.output.open("x", encoding="utf-8", newline="") as destination:
        writer = csv.writer(destination)
        writer.writerow(["log", "phase_start_deg", "phase_end_deg", "covered_s",
                         "mean_window_rad_s", "mean_Uq_V", "slow_fraction", "reverse_fraction"])
        for path in args.logs:
            lines = path.read_text(encoding="utf-8").splitlines()
            reference = next((line for line in lines if line.startswith("RUN_REFERENCE ")), None)
            if reference is None:
                raise SystemExit(f"{path}: no RUN_REFERENCE; calibrated electrical phase unavailable")
            params = {k: float(v) for k, v in re.findall(r"([a-z_]+)=(-?[0-9.]+)", reference)}
            start = next(n for n, line in enumerate(lines) if line.startswith("trace_us,"))
            end = lines.index("TRACE_END", start)
            rows = [{k: float(v) for k, v in row.items()} for row in
                    csv.DictReader(io.StringIO("\n".join(lines[start:end])))]
            bins = [[0.0] * 5 for _ in range(12)]
            for a, b in zip(rows, rows[1:]):
                if a["trace_us"] < 800000:
                    continue
                dt = (b["trace_us"] - a["trace_us"]) / 1e6
                phase = (params["sensor_direction"] * 7 * a["angle_rad"] - params["zero_electric_angle_rad"]) % math.tau
                velocity = params["sensor_direction"] * a["window10ms_rad_s"]
                bucket = bins[min(11, int(phase / math.tau * 12))]
                for n, value in enumerate((1, velocity, a["Uq_V"], abs(velocity) < .2, velocity < -.2)):
                    bucket[n] += dt * value
            for n, bucket in enumerate(bins):
                duration = bucket[0]
                writer.writerow([path.name, n * 30, (n + 1) * 30, duration] +
                                [value / duration if duration else "" for value in bucket[1:]])
            phase = (params["sensor_direction"] * 7 * rows[-1]["angle_rad"] - params["zero_electric_angle_rad"]) % math.tau
            print(f"{path.name}: final calibrated electrical phase={math.degrees(phase):.2f}deg")


if __name__ == "__main__":
    main()
