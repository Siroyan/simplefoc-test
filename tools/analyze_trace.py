"""Summarize the buffered diagnostic trace in a captured experiment log."""
import argparse
import csv
import io
import json
import math
import re
from guard_peak import parse_guard_peaks
from reference_window import comparison_start_us
from speed_range import parse_interval_velocity_stats
from edge_estimator import parse_edge_estimator
from run_edge_log import parse_run_edges
from current_tracking import parse_current_tracking


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument('--start-us',type=int,help='Common comparison start; must follow the reference ramp')
    args = parser.parse_args()
    with open(args.log, encoding="utf-8") as source:
        lines = source.read().splitlines()
    firmware = next((line for line in lines if line.startswith("FIRMWARE ")), None)
    peak_mode=any(line.startswith('RUN CURRENT VELOCITY:') or (line.startswith('CURRENT_PROBE ') and 'stationary_attempt=1' in line) for line in lines)
    peak_required=bool(firmware and int(re.search(r'diag_version=(\d+)',firmware)[1])>=97 and peak_mode)
    guard_peak_snapshots=parse_guard_peaks(lines,required=peak_required)
    analysis_start_us=comparison_start_us(lines,args.start_us)
    interval_velocity_stats=parse_interval_velocity_stats(lines)
    edge_estimator=parse_edge_estimator(lines)
    run_edges=parse_run_edges(lines)
    current_tracking=parse_current_tracking(lines)
    configuration_notes = []
    if analysis_start_us>800000:
        configuration_notes.append(f'Covered metrics begin at {analysis_start_us}us; compare other runs over the same interval after their reference ramps. The legacy after_0_8s key aliases these startup-aware metrics.')
    if firmware == "FIRMWARE diag_version=56":
        configuration_notes.append("Invalid current-LPF comparison: v56 printed requested 1ms but reset both current LPFs to 2ms before drive. Treat this as a 2ms run.")
    is_open = any(line.startswith("RUN OPEN:") for line in lines)
    is_fixed = any(line.startswith("DRIVE fixed_uq_V=") for line in lines)
    is_phase_pulse = any(line.startswith("DRIVE phase_pulse=") for line in lines)
    is_fixed_iq = any(line.startswith("DRIVE fixed_iq_nominal_A=") for line in lines)
    is_current_velocity = any(line.startswith("DRIVE current_velocity=") for line in lines)
    rolling_control = any(line.startswith("ESTIMATOR controller=rolling_LS") for line in lines)
    correction_requested = any(line.startswith("CURRENT_FF_CONFIG ") for line in lines)
    current_probe = next((line for line in lines if line.startswith("CURRENT_PROBE ")), None)
    adaptive_metadata=next((line for line in lines if line.startswith("ADAPTIVE_CONFIG ")),None)
    adaptive_summary=next(({key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=(-?[0-9.]+)",line)} for line in lines if line.startswith("ADAPTIVE_SUMMARY ")),None)
    correction_summary = next(({key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=(-?[0-9.]+)",line)}
                               for line in lines if line.startswith("CURRENT_FF_SUMMARY ")),None)
    reference_line = next((line for line in lines if line.startswith("RUN_REFERENCE ")), None)
    reference = {key: float(value) for key, value in re.findall(r"([a-z_]+)=(-?[0-9.]+)", reference_line or "")}
    displacement_agreement = None
    abi_run_line = next((line for line in lines if line.startswith("ABI_QUALITY_RUN ")), None)
    run_summary_line = next((line for line in lines if line.startswith("RUN_SUMMARY ")), None)
    resolution = next((re.search(r"counts_per_turn=(\d+)", line) for line in lines
                       if line.startswith("CONFIG ") and "counts_per_turn=" in line), None)
    if abi_run_line and run_summary_line and resolution:
        count_delta = float(re.search(r"(?<!_)count_delta=(-?[0-9.]+)", abi_run_line)[1])
        reported = float(run_summary_line.split(": ",1)[1].split(",")[1])
        count_displacement = count_delta*math.tau/int(resolution[1])
        displacement_agreement = {"reported_rad":reported,"count_rad":count_displacement,"difference_rad":reported-count_displacement}
        if abs(reported-count_displacement)>2*math.tau/int(resolution[1]):
            configuration_notes.append("RUN_SUMMARY displacement differs from run ABI counts by over two counts; a cached start angle can include pre-run settling. Common TRACE metrics do not use this start angle.")
    direction = reference.get("sensor_direction")
    direction_source = "RUN_REFERENCE"
    if is_phase_pulse:
        direction = 1
        direction_source = "raw encoder sign; phase pulses have no aligned rotation target"
    if direction is None:
        alignment = next((line for line in lines if line.startswith("ALIGN measured radians forward/reverse:")), None)
        direction = 1 if alignment is None or float(alignment.split(":", 1)[1].split("/")[0]) > 0 else -1
        direction_source = "alignment sign" if alignment else "assumed +1"
    start = next(n for n, line in enumerate(lines) if line.startswith("trace_us,"))
    end = lines.index("TRACE_END", start)
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader(io.StringIO("\n".join(lines[start:end]))) ]
    # This is independent of Encoder::getVelocity, but uses the same ABI angle.
    differences = [(b["angle_rad"] - a["angle_rad"]) * 1e6 /
                   (b["trace_us"] - a["trace_us"])
                   for a, b in zip(rows, rows[1:])]
    last = rows[-1]
    retained_tail=rows[0]['trace_us']>800000
    if retained_tail:
        configuration_notes.append("Only the retained RAM tail is available; covered metrics do not describe the full interval after startup.")
    tail = next(row for row in rows if row["trace_us"] >= last["trace_us"] - 100000)
    # Compare the covered portion after startup; old logs may cover only a tail.
    settled = [row for row in rows if row["trace_us"] >= analysis_start_us]
    command_direction = -1 if settled and sum(row["target_rad_s"] for row in settled)<0 else 1
    metrics = None
    if len(settled) > 1:
        duration_us = settled[-1]["trace_us"] - settled[0]["trace_us"]
        squared_error = window_squared_error = slow_us = negative_us = 0.0
        negative_uq_us = negative_requested_uq_us = 0.0
        voltage_integral = 0.0
        for a, b in zip(settled, settled[1:]):
            dt = b["trace_us"] - a["trace_us"]
            filtered = a["filtered_rad_s"] * (direction if is_open else 1)
            window_speed = a["window10ms_rad_s"] * direction
            squared_error += dt * (filtered - a["target_rad_s"]) ** 2
            window_squared_error += dt * (window_speed - a["target_rad_s"]) ** 2
            slow_us += dt if abs(a["window10ms_rad_s"]) < 0.2 else 0
            negative_us += dt if window_speed * command_direction < -0.2 else 0
            negative_uq_us += dt if a["Uq_V"] < 0 else 0
            negative_requested_uq_us += dt if a.get("unslewed_pid_uq_V", 0) < 0 else 0
            voltage_integral += dt * a["Uq_V"]
        metrics = {
            "covered_start_us": settled[0]["trace_us"],
            "covered_end_us": settled[-1]["trace_us"],
            "mean_encoder_velocity_rad_s": direction * (settled[-1]["angle_rad"] - settled[0]["angle_rad"]) * 1e6 / duration_us,
            "command_direction": command_direction,
            "mean_speed_in_command_direction_rad_s": command_direction * direction * (settled[-1]["angle_rad"] - settled[0]["angle_rad"]) * 1e6 / duration_us,
            "target_magnitude_rad_s": abs(settled[0]["target_rad_s"]) if all(abs(r["target_rad_s"]-settled[0]["target_rad_s"])<1e-6 for r in settled) else None,
            "filtered_tracking_rmse_rad_s": None if is_fixed or is_phase_pulse or is_fixed_iq else math.sqrt(squared_error / duration_us),
            "window_tracking_rmse_rad_s": None if is_fixed or is_phase_pulse or is_fixed_iq else math.sqrt(window_squared_error / duration_us),
            "window_speed_below_0_2_fraction": slow_us / duration_us,
            "reverse_window_speed_below_minus_0_2_fraction": None if is_phase_pulse else negative_us / duration_us,
            "negative_voltage_command_fraction": negative_uq_us / duration_us,
            "voltage_command_mean_V": voltage_integral / duration_us,
            "negative_unslewed_pid_request_fraction": negative_requested_uq_us / duration_us if
                any(line.startswith("LIMITER asymmetric:") for line in lines) and not is_fixed else None,
        }
        target_magnitude=metrics["target_magnitude_rad_s"]
        metrics["relative_window_tracking_rmse"] = metrics["window_tracking_rmse_rad_s"]/target_magnitude if target_magnitude and metrics["window_tracking_rmse_rad_s"] is not None else None
    field_metrics = None
    if reference_line and is_open and "open_field_mechanical_rad" in rows[0]:
        previous_field = reference["open_field_start_rad"]
        field_displacement = 0.0
        errors = []
        for row in rows:
            field_displacement += math.remainder(row["open_field_mechanical_rad"] - previous_field, math.tau)
            previous_field = row["open_field_mechanical_rad"]
            rotor_displacement = reference["sensor_direction"] * (row["angle_rad"] - reference["start_angle_rad"])
            if row["trace_us"] >= 800000:
                errors.append(field_displacement - rotor_displacement)
        field_metrics = {
            "field_displacement_rad": field_displacement,
            "rotor_displacement_in_field_direction_rad": rotor_displacement,
            "final_field_minus_rotor_rad": field_displacement - rotor_displacement,
            "after_0_8s_field_minus_rotor_range_rad": [min(errors), max(errors)] if errors else None,
            "note": "Displacements relative to alignment endpoint; not an independent angle measurement",
        }
    result = {
        "mode": "closed_velocity_current_adaptive14" if adaptive_metadata and "order=14 " in adaptive_metadata else ("closed_velocity_current_adaptive84_zero" if "learning_rate=0.0000 " in adaptive_metadata else "closed_velocity_current_adaptive84_zero_lag" if "response_lag_rad=0.000000 " in adaptive_metadata else "closed_velocity_current_adaptive84") if adaptive_metadata else "stationary_current_probe" if current_probe and is_fixed_iq else ("closed_velocity_current_probe" if current_probe else "closed_velocity_current_index84" if correction_requested else "closed_velocity_current_rolling" if rolling_control else "closed_velocity_current") if is_current_velocity else "fixed_iq_nominal" if is_fixed_iq else "phase_voltage_pulse" if is_phase_pulse else "fixed_uq_sensor_commutation" if is_fixed else "open_loop" if is_open else "closed_forward_only" if "OUTPUT_POLICY forward_only=1" in lines else
                "closed_asymmetric" if any(line.startswith("LIMITER asymmetric:") for line in lines) else "closed_velocity",
        "limiter_metadata": next((line for line in lines if line.startswith("LIMITER ")), None),
        "output_policy": next((line for line in lines if line.startswith("OUTPUT_POLICY ")), None),
        "drive_metadata": next((line for line in lines if line.startswith("DRIVE ")), None),
        "pwm_metadata": next((line for line in lines if line.startswith("PWM ")), None),
        "alignment_profile": next((line for line in lines if line.startswith("ALIGN_PROFILE ")), None),
        "reference_ramp_metadata": next((line for line in lines if line.startswith("REFERENCE_RAMP ")), None),
        "current_observation_metadata": next((line for line in lines if line.startswith("CURRENT_CONFIG ")), None),
        "firmware": firmware,
        "configuration_notes": configuration_notes,
        "displacement_count_agreement": displacement_agreement,
        "current_active": next((line for line in lines if line.startswith("CURRENT_ACTIVE ")), None),
        "current_actuator": next((line for line in lines if line.startswith("CURRENT_ACTUATOR_STATS ")), None),
        "current_probe": current_probe,
        "estimator": " | ".join(line for line in lines if line.startswith(("CONFIG ","ESTIMATOR "))),
        "guard_metadata": next((line for line in lines if line.startswith("STOP_GUARD ")),None),
        "guard_peak_snapshots":guard_peak_snapshots,
        "analysis_start_us":analysis_start_us,
        "interval_velocity_stats":interval_velocity_stats,
        "edge_estimator":edge_estimator,
        "run_edges":run_edges,
        "current_tracking":current_tracking,
        "current_loop": next((line for line in lines if line.startswith("CURRENT_LOOP ")), None),
        "current_engine": next((line for line in lines if line.startswith("CURRENT_ENGINE ")), None),
        "current_slew": next((line for line in lines if line.startswith("CURRENT_SLEW ")), None),
        "current_telemetry": next((line for line in lines if line.startswith("CURRENT_TELEMETRY ")), None),
        "abi_quality_metadata": [line for line in lines if line.startswith("ABI_QUALITY")],
        "abi_quality_run": next(({key: float(value) for key,value in re.findall(r"([a-z_]+)=(-?[0-9.]+)",line)}
                                  for line in lines if line.startswith("ABI_QUALITY_RUN ")),None),
        "index_observed_count": sum(line.startswith("INDEX_REFERENCE ") and "encoder_angle_rad=" in line for line in lines),
        "position_correction_requested": correction_requested,
        "position_correction_summary": correction_summary,
        "index_reference_metadata": [line for line in lines if line.startswith("INDEX_REFERENCE ")],
        "encoder_direction_used": direction,
        "encoder_direction_source": direction_source,
        "samples": len(rows),
        "adaptive_metadata":adaptive_metadata,
        "adaptive_summary":adaptive_summary,
        "metrics_scope": "retained_tail_only" if retained_tail else "covered_after_startup",
        "trace_configuration": " | ".join(line for line in lines if line.startswith(("TRACE_CONFIG ","TRACE_SCOPE "))),
        "trace_span_s": (last["trace_us"] - rows[0]["trace_us"]) / 1e6,
        "last_sample": last,
        "covered_after_0_8s_metrics": metrics,
        "covered_after_startup_metrics": metrics,  # Canonical name; retain the older key for existing consumers.
        "open_field_tracking": field_metrics,
        "control_metadata": next((line for line in lines if line.startswith("CONTROL ")), None),
        "run_summary": next((line for line in lines if line.startswith("RUN_SUMMARY ")), None),
        "stop_reason": next((line for line in lines if line.startswith("STOP:")), None),
        "filtered_range_rad_s": [min(r["filtered_rad_s"] for r in rows), max(r["filtered_rad_s"] for r in rows)],
        "filtered_abs_peak_rad_s": max(abs(r["filtered_rad_s"]) for r in rows),
        "window_abs_peak_rad_s": max(abs(r["window10ms_rad_s"]) for r in rows),
        "max_sampled_loop_us_after_0_8s": max((r["loop_us"] for r in rows if r["trace_us"] >= 800000), default=None),
        "window_age_range_us": [min(r["window_age_us"] for r in rows),max(r["window_age_us"] for r in rows)] if "window_age_us" in rows[0] else None,
        "Uq_range_V": [min(r["Uq_V"] for r in rows), max(r["Uq_V"] for r in rows)],
        "raw_range_rad_s": [min(r["raw_rad_s"] for r in rows), max(r["raw_rad_s"] for r in rows)],
        "window10ms_range_rad_s": [min(r["window10ms_rad_s"] for r in rows), max(r["window10ms_rad_s"] for r in rows)],
        "trace_angle_difference_range_rad_s": [min(differences), max(differences)],
        "last_about_100ms_mean_rad_s": (last["angle_rad"] - tail["angle_rad"]) * 1e6 /
                                            (last["trace_us"] - tail["trace_us"]),
        "host_stop_acknowledged": "HOST final_stop_acknowledged=True" in lines,
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
