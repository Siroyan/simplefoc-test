"""Compare captured closed-loop trials; no hardware access."""
import glob
import argparse
import csv
import io
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("patterns", nargs="+")
    parser.add_argument("--csv", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    paths = sorted({Path(path) for pattern in args.patterns for path in glob.glob(pattern)})
    if not paths:
        raise SystemExit("Provide one or more log paths or glob patterns")
    analyzer = Path(__file__).with_name("analyze_trace.py")
    results = []
    for path in paths:
        result = json.loads(subprocess.check_output([sys.executable, str(analyzer), str(path)], text=True))
        summary = result["run_summary"]
        elapsed, angle = map(float, summary.split(": ", 1)[1].split(","))
        results.append({
            "log": path.name,
            "mode": result["mode"],
            "limiter": result["limiter_metadata"],
            "output_policy": result["output_policy"],
            "drive": result["drive_metadata"],
            "alignment_profile": result["alignment_profile"],
            "reference_ramp": result["reference_ramp_metadata"],
            "adaptive_config":result["adaptive_metadata"],
            "adaptive_summary":result["adaptive_summary"],
            "metrics_scope":result["metrics_scope"],
            "trace_span_s":result["trace_span_s"],
            "trace_configuration":result["trace_configuration"],
            "pwm": result["pwm_metadata"],
            "current_observation": result["current_observation_metadata"],
            "abi_quality_run": result["abi_quality_run"],
            "index_observed_count": result["index_observed_count"],
            "position_correction_requested": result["position_correction_requested"],
            "position_correction_summary": result["position_correction_summary"],
            "guard_metadata":result["guard_metadata"],
            "guard_peak_snapshots":result["guard_peak_snapshots"],
            "interval_velocity_stats":result["interval_velocity_stats"],
            "edge_estimator":result["edge_estimator"],
            "run_edges":{key:value for key,value in result["run_edges"].items() if key!='rows'} if result["run_edges"] else None,
            "current_tracking":result["current_tracking"],
            "current_active":result["current_active"],
            "current_actuator":result["current_actuator"],
            "current_probe":result["current_probe"],
            "configuration_notes":result["configuration_notes"],
            "displacement_count_agreement":result["displacement_count_agreement"],
            **{key: result[key] for key in ("firmware", "estimator", "current_loop", "current_engine", "current_slew", "current_telemetry", "max_sampled_loop_us_after_0_8s")},
            "open_field_tracking": result["open_field_tracking"],
            "control": result["control_metadata"],
            "stop": result["stop_reason"],
            "duration_s": elapsed / 1e6,
            "displacement_rad": angle,
            "mean_speed_rad_s": angle / (elapsed / 1e6),
            "covered_after_0_8s_metrics": result["covered_after_0_8s_metrics"],
            "analysis_start_us": result["analysis_start_us"],
            "covered_after_startup_metrics": result["covered_after_startup_metrics"],
            "captured_filtered_abs_peak_rad_s": result["filtered_abs_peak_rad_s"],
            "captured_window_abs_peak_rad_s": result["window_abs_peak_rad_s"],
            "Uq_range_V": result["Uq_range_V"],
            "final_stop_acknowledged": result["host_stop_acknowledged"],
        })
    if args.csv:
        buffer = io.StringIO(newline="")
        writer = csv.writer(buffer)
        writer.writerow(["log", "mode", "duration_s", "displacement_rad", "control", "limiter", "drive", "pwm", "current_observation",
                         "metrics_start_s", "metrics_end_s", "mean_after_startup_rad_s",
                         "window_tracking_rmse_rad_s", "slow_fraction", "reverse_fraction", "window_abs_peak_rad_s",
                         "negative_voltage_fraction", "negative_requested_voltage_fraction",
                         "stop_reason", "final_stop_acknowledged", "firmware", "estimator", "current_loop", "current_engine", "current_slew", "current_telemetry", "max_sampled_loop_us_after_0_8s",
                         "abi_run_invalid", "abi_run_duplicate", "abi_net_counts", "encoder_net_counts", "index_references", "correction_requested", "correction_active_calls", "correction_peak_A", "correction_invalid", "guard", "current_active", "configuration_notes", "current_actuator", "current_probe", "run_reference_minus_ABI_displacement_rad", "command_direction", "mean_speed_in_command_direction_rad_s", "alignment_profile", "metrics_scope", "trace_span_s", "trace_configuration", "adaptive_config", "adaptive_summary", "target_magnitude_rad_s", "relative_window_tracking_rmse", "reference_ramp", "guard_peak_snapshots", "interval_velocity_stats", "edge_estimator", "run_edges", "current_tracking"])
        for row in results:
            metrics = row["covered_after_0_8s_metrics"] or {}
            abi = row["abi_quality_run"] or {}
            correction = row["position_correction_summary"] or {}
            writer.writerow([row["log"], row["mode"], row["duration_s"], row["displacement_rad"],
                             row["control"], row["limiter"], row["drive"], row["pwm"], row["current_observation"],
                             metrics["covered_start_us"] / 1e6 if metrics else "",
                             metrics["covered_end_us"] / 1e6 if metrics else "",
                             metrics.get("mean_encoder_velocity_rad_s", ""),
                             metrics.get("window_tracking_rmse_rad_s", ""),
                             metrics.get("window_speed_below_0_2_fraction", ""),
                             metrics.get("reverse_window_speed_below_minus_0_2_fraction", ""),
                             row["captured_window_abs_peak_rad_s"],
                             metrics.get("negative_voltage_command_fraction", ""),
                             metrics.get("negative_unslewed_pid_request_fraction", ""),
                             row["stop"], row["final_stop_acknowledged"],
                             *[row[key] for key in ("firmware", "estimator", "current_loop", "current_engine", "current_slew", "current_telemetry", "max_sampled_loop_us_after_0_8s")],
                             *[abi.get(key, "") for key in ("invalid", "duplicate", "count_delta", "encoder_count_delta")], row["index_observed_count"],
                             row["position_correction_requested"], *[correction.get(key, "") for key in ("active_calls", "peak_requested_A", "invalid")],row["guard_metadata"],row["current_active"]," | ".join(row["configuration_notes"]),row["current_actuator"],row["current_probe"],(row["displacement_count_agreement"] or {}).get("difference_rad", ""),metrics.get("command_direction", ""),metrics.get("mean_speed_in_command_direction_rad_s", ""),row["alignment_profile"],row["metrics_scope"],row["trace_span_s"],row["trace_configuration"],row["adaptive_config"],json.dumps(row["adaptive_summary"],sort_keys=True) if row["adaptive_summary"] else "",metrics.get("target_magnitude_rad_s", ""),metrics.get("relative_window_tracking_rmse", ""),row["reference_ramp"],json.dumps(row["guard_peak_snapshots"],sort_keys=True) if row["guard_peak_snapshots"] else "",json.dumps(row["interval_velocity_stats"],sort_keys=True) if row["interval_velocity_stats"] else "",json.dumps(row["edge_estimator"],sort_keys=True) if row["edge_estimator"] else "",json.dumps(row["run_edges"],sort_keys=True) if row["run_edges"] else "",json.dumps(row["current_tracking"],sort_keys=True) if row["current_tracking"] else ""])
        output = buffer.getvalue()
    else:
        output = json.dumps(results, indent=2) + "\n"
    if args.output:
        # csv.writer already supplies CRLF; avoid an extra CR on Windows.
        with args.output.open("w", encoding="utf-8", newline="") as destination:
            destination.write(output)
        print(f"Wrote {len(results)} trial summaries to {args.output}")
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
