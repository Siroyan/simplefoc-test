"""Analyze a disabled hand-turn log without accessing hardware."""
import argparse
import json
import math
import re
from pathlib import Path
from statistics import median


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--configured-counts-per-turn", type=int,
                        help="Override CONFIG metadata; legacy logs default to 4096")
    args = parser.parse_args()
    lines = args.log.read_text(encoding="utf-8").splitlines()
    configured_cpr = args.configured_counts_per_turn
    if configured_cpr is None:
        match = re.search(r"counts_per_turn=(\d+)", "\n".join(lines))
        configured_cpr = int(match[1]) if match else 4096
    rows = [list(map(float, line.split(",")[1:]))
            for line in lines if line.startswith("MANUAL,")]
    start = lines.index("INDEX us,A_edges,B_edges,signed_count") + 1
    end = lines.index("EDGES_LAST us,signed_count,AB_state", start)
    indices = [list(map(int, line.split(","))) for line in lines[start:end]]
    turns = [{"elapsed_s": (b[0] - a[0]) / 1e6,
              "A_edges": b[1] - a[1], "B_edges": b[2] - a[2],
              "signed_counts": b[3] - a[3]}
             for a, b in zip(indices, indices[1:])]
    # Locate the index count in 50Hz telemetry by interpolating the first
    # forward crossing. No constant-speed assumption over the turn is needed.
    # Timing within the bracketing telemetry interval is only approximate.
    crossing_times = []
    cursor = 0
    for index in indices:
        for n in range(cursor, len(rows) - 1):
            a, b = rows[n:n + 2]
            if a[5] < index[3] <= b[5]:
                fraction = (index[3] - a[5]) / (b[5] - a[5])
                crossing_times.append(a[0] + fraction * (b[0] - a[0]))
                cursor = n + 1
                break
        else:
            raise ValueError("Could not locate forward index count crossing in telemetry")

    def integral(column, start_ms, end_ms):
        total = 0.0
        for a, b in zip(rows, rows[1:]):
            lo, hi = max(a[0], start_ms), min(b[0], end_ms)
            if hi <= lo:
                continue
            width = b[0] - a[0]
            left = a[column] + (b[column] - a[column]) * (lo - a[0]) / width
            right = a[column] + (b[column] - a[column]) * (hi - a[0]) / width
            total += (left + right) * 0.5 * (hi - lo) / 1000
        return total

    for turn, start_ms, end_ms in zip(turns, crossing_times, crossing_times[1:]):
        configured_angle = turn["signed_counts"] * math.tau / configured_cpr
        turn["approximate_telemetry_crossing_ms"] = [start_ms, end_ms]
        turn["count_angle_with_recorded_configuration_rad"] = configured_angle
        turn["filtered_velocity_integral_rad"] = integral(3, start_ms, end_ms)
        turn["window_velocity_integral_rad"] = integral(4, start_ms, end_ms)
        turn["filtered_integral_to_count_angle_ratio"] = turn["filtered_velocity_integral_rad"] / configured_angle
    # Approximately 200ms angle differences average hand-speed fluctuations and
    # avoid reusing the stateful Encoder::getVelocity estimator.
    comparisons = []
    for i in range(10, len(rows)):
        now, before = rows[i], rows[i - 10]
        dt = (now[0] - before[0]) / 1000
        velocity = (now[1] - before[1]) / dt
        if abs(velocity) >= 0.5:
            comparisons.append({"ms": now[0], "raw": now[2], "filtered": now[3],
                                "angle_difference_200ms": velocity,
                                "ratio": now[3] / velocity})
    moving = [r for r in rows if abs(r[4]) >= 0.2]
    edge_start = end + 1
    edge_end = next(n for n in range(edge_start, len(lines)) if lines[n].startswith("MANUAL_END:"))
    edges = [list(map(int, line.split(","))) for line in lines[edge_start:edge_end]]
    edge_intervals = [b[0] - a[0] for a, b in zip(edges, edges[1:])]
    # Replay the local SimpleFOC 2.4.0 mixed time/frequency estimator on the
    # saved edge times. Fixed 28us calls are an explanatory simulation, not
    # a reconstruction of the actual hand-turn loop (UART changes its timing).
    forward = [edges[0]]
    for edge in edges[1:]:
        if edge[1] <= forward[-1][1]:
            break
        forward.append(edge)
    current = forward[0]
    previous_count = current[1]
    previous_age = 0.0
    pulses_per_second = 0.0
    integrated_count = 0.0
    at = 1
    for timestamp in range(current[0] + 28, forward[-1][0] + 1, 28):
        while at < len(forward) and forward[at][0] <= timestamp:
            current = forward[at]
            at += 1
        age = (timestamp - current[0]) * 1e-6
        dt = 28e-6 + previous_age - age
        dn = current[1] - previous_count
        if dn and dt > 14e-6:
            pulses_per_second = dn / dt
        if age > 0.1:
            pulses_per_second = 0.0
        integrated_count += pulses_per_second * 28e-6
        previous_count = current[1]
        previous_age = age
    actual_count = current[1] - forward[0][1]
    result = {
        "configured_counts_per_turn": configured_cpr,
        "firmware_integrals": next((line for line in lines if line.startswith("MANUAL_INTEGRALS")), None),
        "telemetry_rows": len(rows), "index_records": indices, "index_intervals": turns,
        "whole_capture_integrals": {
            "angle_change_rad": rows[-1][1] - rows[0][1],
            "filtered_velocity_rad": integral(3, rows[0][0], rows[-1][0]),
            "window_velocity_rad": integral(4, rows[0][0], rows[-1][0]),
            "note": "50Hz trapezoidal approximation; includes stationary periods at both ends",
        },
        "last_telemetry": rows[-1],
        "max_abs_raw_rad_s": max(abs(r[2]) for r in rows),
        "max_abs_filtered_rad_s": max(abs(r[3]) for r in rows),
        "moving_filtered_to_200ms_ratio_median": median(r["ratio"] for r in comparisons),
        "moving_filtered_over5_while_window_under5_rows": sum(abs(r[3]) > 5 and abs(r[4]) < 5 for r in moving),
        "largest_filtered_rows_with_200ms_comparison": sorted(comparisons, key=lambda r: abs(r["filtered"]), reverse=True)[:5],
        "last_edge_intervals_us_min_median_max": [min(edge_intervals), median(edge_intervals), max(edge_intervals)],
        "illustrative_28us_estimator_replay": {
            "forward_events": len(forward), "actual_net_counts": actual_count,
            "integral_of_estimated_counts": integrated_count,
            "estimated_to_actual_distance_ratio": integrated_count / actual_count,
            "note": "Explanatory simulation on recorded edges; not actual loop timing",
        },
        "summary": next(line for line in lines if line.startswith("MANUAL_SUMMARY")),
        "final_stop_acknowledged": "HOST final_stop_acknowledged=True" in lines,
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
