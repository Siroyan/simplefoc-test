"""Capture one explicitly requested test; never reset or retry the board."""
import argparse
from datetime import datetime
from pathlib import Path
import re
import time

import serial


def verify_active_filters(line, current_lpf, velocity_lpf, q_p=None, q_i=None, velocity_p=None, velocity_i=None):
    values = {key: float(value) for key, value in re.findall(r"([A-Za-z_]+)=([0-9.]+)", line)}
    expected = {"q_LPF_s": current_lpf, "d_LPF_s": current_lpf, "velocity_LPF_s": velocity_lpf}
    if q_p is not None or q_i is not None:
        expected.update({"q_P":q_p,"q_I":q_i})
    if velocity_p is not None or velocity_i is not None:
        expected.update({"velocity_P":velocity_p,"velocity_I":velocity_i})
    if any(value is None or value <= 0 or abs(values.get(key, -1)-value) > 1e-6 for key, value in expected.items()):
        raise RuntimeError("Active current/velocity filter settings differ from the requested profile; stopping")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--command", choices=("t", "o", "a", "b", "d", "j", "k", "r", "n", "m", "f", "g", "w", "z", "y", "q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "e", "l", "p", "u", "v"), required=True)
    parser.add_argument("--current-limit", type=float, required=True,
                        help="User-confirmed supply current limit in A (metadata only)")
    parser.add_argument('--expect-response-hz',type=float,choices=(13.5,17.5),default=13.5,
                        help='Expected frequency for the T/U response identification only')
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quiet", action="store_true", help="Keep the complete raw log; print only run/status metadata")
    parser.add_argument("--expect-pwm-hz", type=int, choices=(25000, 12500),
                        help="Query c and verify timer period, clock and unchanged 800ns dead time before drive")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents overwriting experiment evidence.
    with args.output.open("x", encoding="utf-8", buffering=1) as log:
        def record(line):
            if not args.quiet or line.startswith(("HOST", "STOP", "RUN", "ALIGN", "CONTROL", "PWM", "CONFIG",
                                                  "LIMITER", "OUTPUT", "DRIVE", "CURRENT", "SENSE", "FIRMWARE", "ESTIMATOR", "INDEX_REFERENCE", "ABI_QUALITY", "Rejected:", "ERROR:", "TRACE_END")):
                print(line, flush=True)
            log.write(line + "\n")

        record(f"HOST start={datetime.now().astimezone().isoformat()} port={args.port}")
        record(f"HOST supply=11.1V current_limit={args.current_limit}A (user specified); load removed, motor fixed; CV unmeasured")
        link = serial.Serial(port=None, baudrate=115200, timeout=0.2)
        link.dtr = False
        link.rts = False
        link.port = args.port
        link.open()
        sent = False
        trace_end = False
        alignment_dump = False
        stopped = False
        off_rows = 0
        firmware_version = 0
        current_lpf = velocity_lpf = None
        velocity_p=velocity_i=None
        adaptive_config_verified=False
        response_probe_config_verified=False
        q_p = q_i = None
        active_filters_confirmed = False
        buffer = b""

        def read_lines():
            nonlocal buffer
            buffer += link.read(max(1, link.in_waiting))
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    record(text)
                    yield text

        try:
            # Flush old data and establish live, disabled telemetry before commanding.
            link.reset_input_buffer()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and off_rows < 3:
                for line in read_lines():
                    parts = line.split(",")
                    if len(parts) == 8 and parts[0].isdigit():
                        if parts[1] != "0" or float(parts[7]) != 0:
                            raise RuntimeError("Board is not output-OFF; aborting")
                        off_rows += 1
            if off_rows < 3:
                raise RuntimeError("No live disabled telemetry; test not started")
            if args.expect_pwm_hz is not None:
                link.write(b"c")
                link.flush()
                verified_pwm = False
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline and not verified_pwm:
                    for line in read_lines():
                        if not line.startswith("PWM "):
                            continue
                        values = {k: float(v) for k, v in re.findall(r"([a-zA-Z0-9_]+)=([0-9.]+)", line)}
                        expected_arr = 170000000 // (2 * args.expect_pwm_hz) - 1
                        if (values.get("hz") != args.expect_pwm_hz or values.get("system_clock_Hz") != 170000000 or
                                values.get("tim1_psc") != 0 or values.get("tim1_arr") != expected_arr or
                                int(values.get("tim1_bdtr", 0)) & 255 != 0x84 or
                                int(values.get("tim1_cr1", 0)) & 0x300 != 0 or
                                int(values.get("tim1_cr1", 0)) & 0x60 != 0x40 or values.get("rcc_cfgr") != 15 or
                                ("gate_inputs_low" in values and values["gate_inputs_low"] != 1)):
                            raise RuntimeError("PWM/dead-time invariant mismatch; drive not started")
                        verified_pwm = True
                if not verified_pwm:
                    raise RuntimeError("No PWM register reply; drive not started")
                record("HOST verified PWM period/clock and unchanged DTG=0x84 (800ns nominal); gates remain OFF")
            record(f"HOST sending one command={args.command} at={datetime.now().astimezone().isoformat()}")
            link.write(args.command.encode("ascii"))
            link.flush()
            sent = True
            off_rows = 0
            # j adds a 2s ADC zero calibration and a larger dump AFTER output OFF.
            deadline = time.monotonic() + (35 if args.command=="C" else 30 if args.command in ("j", "r", "n", "m", "f", "g", "w", "z", "y", "q", "Q", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "e", "l") else 25)
            while time.monotonic() < deadline:
                for line in read_lines():
                    if line.startswith("FIRMWARE diag_version="):
                        firmware_version = int(line.split("=", 1)[1])
                    if line.startswith("CURRENT_PROBE ") and args.command in ("T", "U"):
                        config={key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=([0-9.]+)",line)}
                        expected={"frequency_Hz":args.expect_response_hz,"phase_rad":0 if args.command=="T" else 3.14159265359,"amplitude_nominal_A":.02,"start_us":800000,"blend_us":200000,"total_bound_A":.45,"stationary_attempt":0}
                        if any(abs(config.get(key,-1)-value)>1e-6 for key,value in expected.items()):
                            raise RuntimeError("Velocity-response probe configuration mismatch; stopping")
                        response_probe_config_verified=True
                    if line.startswith("ADAPTIVE_CONFIG ") and args.command in ("T", "U"):
                        raise RuntimeError("Adaptive correction must be disabled during response identification; stopping")
                    if line.startswith("ADAPTIVE_CONFIG ") and args.command in ("A", "Z", "F"):
                        config={key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=([0-9.]+)",line)}
                        expected={"order":84,"learning_rate":0 if args.command=="Z" else .2,"correction_bound_nominal_A":.03,"response_lag_rad":0 if args.command=="F" else .785398,"gate_us":800000,"blend_us":200000,"total_limit_nominal_A":.45,"per_run_reset":1}
                        if any(abs(config.get(key,-1)-value)>1e-6 for key,value in expected.items()):
                            raise RuntimeError("Adaptive comparison configuration mismatch; stopping")
                        adaptive_config_verified=True
                    if line.startswith("TRACE_CONFIG ") and args.command in ("D", "E", "M", "N") and not line.startswith("TRACE_CONFIG period_us=2000 capacity=320 "):
                        raise RuntimeError("Dense trace cadence/capacity mismatch; stopping")
                    if line.startswith("ESTIMATOR controller=rolling_LS") and args.command in ("g", "w", "q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F"):
                        expected_horizon=8 if args.command=="H" else 4
                        if not line.startswith(f"ESTIMATOR controller=rolling_LS{expected_horizon}ms ") or "guard=original_window2ms_LPF2ms guard_limit_rad_s=5" not in line:
                            raise RuntimeError("Rolling estimator or independent speed guard metadata mismatch; stopping")
                    if line.startswith("CURRENT_LOOP "):
                        current_lpf = float(re.search(r"current_LPF_s=([0-9.]+)", line)[1])
                        q_p = float(re.search(r"q_P=([0-9.]+)",line)[1])
                        q_i = float(re.search(r"q_I=([0-9.]+)",line)[1])
                        if firmware_version>=78 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F"):
                            cap=float(re.search(r"vector_voltage_limit_V=([0-9.]+)",line)[1])
                            if abs(q_p-.60)>1e-7 or abs(q_i-200)>1e-7 or abs(cap-.5)>1e-7 or abs(current_lpf-.002)>1e-7:
                                raise RuntimeError("Candidate current PI/filter/vector bound mismatch; stopping")
                    if line.startswith("DRIVE current_velocity=1 ") and firmware_version>=78 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F"):
                        target=float(re.search(r"speed_target_rad_s=(-?[0-9.]+)",line)[1])
                        limit=float(re.search(r"nominal_iq_limit_A=([0-9.]+)",line)[1])
                        expected_target=-1 if args.command in ("Q", "C", "R", "S") else 1
                        if abs(target-expected_target)>1e-7 or abs(limit-.45)>1e-7:
                            raise RuntimeError("Candidate direction/speed/current-setpoint bound mismatch; stopping")
                    if line.startswith("CONTROL "):
                        velocity_lpf = float(re.search(r"velocity_LPF_s=([0-9.]+)", line)[1])
                        velocity_p=float(re.search(r"(?:^| )P=([0-9.]+)",line)[1])
                        velocity_i=float(re.search(r"(?:^| )I=([0-9.]+)",line)[1])
                        if firmware_version>=76 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F"):
                            vp=float(re.search(r"(?:^| )P=([0-9.]+)",line)[1])
                            vi=float(re.search(r"(?:^| )I=([0-9.]+)",line)[1])
                            if abs(vp-(.10 if args.command=="E" else .15))>1e-7 or abs(vi-(9.60 if args.command in ("B", "S", "N") else 7.20 if args.command in ("I", "R", "M", "A", "Z", "T", "U", "F") else 4.80))>1e-7:
                                raise RuntimeError("Candidate velocity P/I mismatch; stopping")
                        if firmware_version>=74 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F") and abs(velocity_lpf-(.001 if args.command=="L" else .002))>1e-7:
                            raise RuntimeError("Candidate velocity-filter configuration mismatch; stopping")
                    if line.startswith("ALIGN_PROFILE ") and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F"):
                        expected_cycles=4 if args.command=="C" else 3
                        expected_deadline=20000000 if args.command=="C" else 0
                        if not line.startswith(f"ALIGN_PROFILE cycles={expected_cycles} ") or f"max_alignment_us={expected_deadline}" not in line:
                            raise RuntimeError("Alignment profile mismatch; stopping")
                    if line.startswith("CURRENT_ACTIVE "):
                        verify_active_filters(line, current_lpf, velocity_lpf, q_p if firmware_version>=65 else None, q_i if firmware_version>=65 else None, velocity_p if firmware_version>=81 else None,velocity_i if firmware_version>=81 else None)
                        active_filters_confirmed = True
                        record("HOST verified active current/velocity filter settings")
                    if firmware_version >= 58 and line.startswith(("RUN CURRENT VELOCITY:", "RUN CURRENT:")) and not active_filters_confirmed:
                        raise RuntimeError("Current control started without active filter metadata; stopping")
                    if line.startswith("RUN CURRENT VELOCITY:") and args.command in ("A", "Z", "F") and not adaptive_config_verified:
                        raise RuntimeError("Adaptive comparison started without bounded profile metadata; stopping")
                    if line.startswith("RUN CURRENT VELOCITY:") and args.command in ("T", "U") and not response_probe_config_verified:
                        raise RuntimeError("Velocity-response probe started without bounded metadata; stopping")
                    if line.startswith(("Rejected:", "ERROR:")):
                        raise RuntimeError(line)
                    if line.startswith("STOP:"):
                        stopped = True
                    if line.startswith("ALIGN_TRACE_BEGIN:"):
                        alignment_dump = True
                    if line == "TRACE_END":
                        trace_end = True
                        alignment_dump = False
                    parts = line.split(",")
                    if stopped and not alignment_dump and len(parts) == 8 and parts[0].isdigit() and parts[1] == "0":
                        if float(parts[7]) != 0:
                            raise RuntimeError("Nonzero voltage command after stop")
                        off_rows += 1
                if stopped and trace_end and off_rows >= 3:
                    record("HOST completed: STOP + TRACE_END + disabled telemetry confirmed")
                    return
                if stopped and not trace_end and off_rows >= 3:
                    raise RuntimeError("Stopped before run trace (alignment or initialization failure)")
            raise RuntimeError("Test timeout; sending stop")
        finally:
            # Also stop on host errors or Ctrl-C, and retain the acknowledgement.
            if link.is_open:
                link.write(b"s")
                link.flush()
                record(f"HOST sent final s; test_command_sent={sent}")
                deadline = time.monotonic() + 2
                acknowledged = False
                while time.monotonic() < deadline:
                    for line in read_lines():
                        if line == "STOP: serial stop":
                            acknowledged = True
                record(f"HOST final_stop_acknowledged={acknowledged}")
                link.close()


if __name__ == "__main__":
    main()
