"""Capture one explicitly requested test; never reset or retry the board."""
import argparse
from datetime import datetime
from pathlib import Path
import re
import time

import serial
from guard_peak import verify_guard_peak_config
from current_tracking import verify_current_tracking
from speed_range import verify_speed_guard_config,verify_velocity_stats_config
from edge_estimator import ESTIMATOR as EDGE_ESTIMATOR, verify_edge_estimator_config,verify_edge_clock_check
from run_edge_log import verify_run_edge_config,verify_run_edge_active,verify_run_edge_observer,verify_run_edge_alignment,OPEN as RAW_OPEN,HIGH_OPEN as RAW_HIGH_OPEN,HIGH_ESTIMATOR,HIGH_DRIVE,FAST_DRIVE
ADAPTIVE_COMMANDS=frozenset('AZF7890?^')
HIGH_RANGE_COMMANDS=frozenset('G134=><~[]{_@%:;()?^')
DWT_EDGE_COMMANDS=frozenset(']{_@%:;()?^')
DWT_CLOCK_COMMANDS=DWT_EDGE_COMMANDS | frozenset('}!+')
REFERENCE_GUARD_COMMANDS=HIGH_RANGE_COMMANDS | frozenset('!+')


def verify_active_filters(line, current_lpf, velocity_lpf, q_p=None, q_i=None, velocity_p=None, velocity_i=None):
    values = {key: float(value) for key, value in re.findall(r"([A-Za-z_]+)=([0-9.]+)", line)}
    expected = {"q_LPF_s": current_lpf, "d_LPF_s": current_lpf, "velocity_LPF_s": velocity_lpf}
    if q_p is not None or q_i is not None:
        expected.update({"q_P":q_p,"q_I":q_i})
    if velocity_p is not None or velocity_i is not None:
        expected.update({"velocity_P":velocity_p,"velocity_I":velocity_i})
    if any(value is None or value <= 0 or abs(values.get(key, -1)-value) > 1e-6 for key, value in expected.items()):
        raise RuntimeError("Active current/velocity filter settings differ from the requested profile; stopping")


def candidate_reference_duration(command):
    if command in ('_','@','%',':',';','(',')','?','^','+'):return 800000
    return 2000000 if command in HIGH_RANGE_COMMANDS or command=='!' else 1500000 if command=='0' else 800000


def candidate_current_lpf(command):
    return .0005 if command=='<' else .001 if command in ('1','3','4','=','>','~','[',']','{','_','@','%',':',';','(',')','?','^') else .002


def verify_rolling_profile(line,command):
    if command in ('[',']','{','_','@','%',':',';','(',')','?','^'):
        if line!=EDGE_ESTIMATOR:raise RuntimeError('Wrong edge-estimator or independent guard profile; stopping')
        return
    horizon,minimum,warmup=(2,75,1500) if command in ('3','4','=','>','<','~','[',']','{','_','@','%',':',';','(',')','?','^') else (8,300,6000) if command=='H' else (4,150,3000)
    prefix=f'ESTIMATOR controller=rolling_LS{horizon}ms min_sample_us={minimum} warmup_span_us={warmup} '
    guard='guard=original_window2ms_LPF2ms guard_limit_rad_s=8.5 guard_profile=reference_envelope' if command in HIGH_RANGE_COMMANDS else 'guard=original_window2ms_LPF2ms guard_limit_rad_s=5'
    if not line.startswith(prefix) or guard not in line:
        raise RuntimeError('Rolling estimator or independent speed guard metadata mismatch; stopping')


def verify_reference_profile(line,expected_target=2,expected_duration=800000):
    fields={key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=([0-9.]+)",line)}
    if fields.get("duration_us")!=expected_duration or abs(fields.get("target_rad_s",-999)-expected_target)>1e-6 or "trace_target=final_goal origin=trace_start" not in line:
        raise RuntimeError("Speed-reference ramp mismatch; stopping")


def verify_response_probe(line,command,frequency=13.5):
    if command not in ('T','U','(',')'):
        raise ValueError('Not a response probe profile')
    config={key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=([0-9.]+)",line)}
    expected={"frequency_Hz":17.5 if command in ('(',')') else frequency,
              "phase_rad":0 if command in ('T','(') else 3.14159265359,
              "amplitude_nominal_A":.02,"start_us":800000,"blend_us":200000,
              "total_bound_A":.45,"stationary_attempt":0,"excitation_only":1}
    if any(abs(config.get(key,-1)-value)>1e-6 for key,value in expected.items()):
        raise RuntimeError('Velocity-response probe configuration mismatch; stopping')


def verify_adaptive_profile(line,command):
    if command not in ADAPTIVE_COMMANDS:
        raise ValueError('Not an adaptive comparison command')
    config={key:float(value) for key,value in re.findall(r"([A-Za-z_]+)=([0-9.]+)",line)}
    expected={"order":84,"learning_rate":0 if command in ('Z','8','0') else .2,
              "correction_bound_nominal_A":.03,"response_lag_rad":0 if command=='F' else .785398,
              "gate_us":800000,"blend_us":200000,"total_limit_nominal_A":.45,"per_run_reset":1}
    if command in ('?','^'):
        expected.update({'order':14,'correction_bound_nominal_A':.01,
                         'learning_reference_rad_s':6.2832,'learning_min_velocity_rad_s':1.8850,'learning_max_velocity_rad_s':10.6814})
    elif command=='9':
        expected.update({'learning_reference_rad_s':3,'learning_min_velocity_rad_s':.9,'learning_max_velocity_rad_s':5.1})
    elif command in ('7','8','0'):
        expected.update({'learning_reference_rad_s':2.5,'learning_min_velocity_rad_s':.75,'learning_max_velocity_rad_s':4.25})
    elif any(key in config for key in ('learning_reference_rad_s','learning_min_velocity_rad_s','learning_max_velocity_rad_s')):
        expected.update({'learning_reference_rad_s':1,'learning_min_velocity_rad_s':.3,'learning_max_velocity_rad_s':1.7})
    if any(abs(config.get(key,-1)-value)>1e-6 for key,value in expected.items()):
        raise RuntimeError('Adaptive comparison configuration mismatch; stopping')
    if command=='^' and ' trig=phase_rotation' not in line:
        raise RuntimeError('Missing optimized harmonic phase calculation; stopping')


def candidate_velocity_profile(command):
    if command not in set('qQCHLDEIRBSMNAZTUFVWJKXYOP2567890G134=><~[]{_@%:;()?^'):
        raise ValueError('Not a candidate velocity profile')
    p_gain,i_gain,lpf,target=.15,4.8,.002,1
    if command in ('I','R','M','A','Z','T','U','F','V','W','Y','O','P','2','5','6','7','8','9','0','G','1','3','4','=','>','<','~','[',']','{','_','@','%',':',';','(',')','?','^'):
        i_gain=7.2
    if command in ('B','S','N'):
        i_gain=9.6
    if command=='V':
        p_gain=.20
    elif command in ('E','K','X'):
        p_gain=.10
    if command in ('L','Y','O','P','5','6','7','8','9','0','G','1','3','4','=','>','<','~','[',']','{','_','@','%',':',';','(',')','?','^'):
        lpf=.001
    if command in ('2','3','4','=','>','<','~','[',']','{','_','@','%',':',';','(',')','?','^'):
        lpf=.0005
    if command in ('3','4','=','>','<','~','[',']','{','_','@','%',':',';','(',')','?','^'):
        p_gain,i_gain=.075 if command in ('{','%') else .12 if command in ('~','[',']','{','_','@','%',':',';','(',')','?','^') else .15 if command in ('>','<') else .075 if command=='=' else .025,.3 if command=='3' else 1.2
    if command in ('Q','C','R','S'):
        target=-1
    elif command in ('W','J','K','Y','6'):
        target=2
    elif command in ('O','2','5','7','8','0'):
        target=2.5
    elif command in HIGH_RANGE_COMMANDS:
        target=6.283185
    elif command in ('X','P','9'):
        target=3
    return p_gain,i_gain,lpf,target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--command", choices=("t", "o", "a", "b", "d", "j", "k", "r", "n", "m", "f", "g", "w", "z", "y", "q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "}", "!", "+", "_", "@", "%", ":", ";", "(", ")", "?", "^", "e", "l", "p", "u", "v"), required=True)
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
                                                  "LIMITER", "OUTPUT", "DRIVE", "CURRENT", "ADAPTIVE", "SENSE", "FIRMWARE", "ESTIMATOR", "EDGE_", "RUN_EDGE", "REFERENCE", "GUARD", "SPEED", "VELOCITY", "INDEX_REFERENCE", "ABI_QUALITY", "Rejected:", "ERROR:", "TRACE_END")):
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
        reference_profile_confirmed=False
        edge_estimator_confirmed=False
        edge_mode_confirmed=False
        edge_clock_confirmed=False
        raw_config_confirmed=raw_active_confirmed=raw_observer_confirmed=raw_end=False
        raw_high_estimator_confirmed=raw_high_drive_confirmed=False
        raw_alignment_confirmed=False
        tracking_config_confirmed=tracking_active_confirmed=False
        guard_peak_config_confirmed=False
        speed_guard_confirmed=velocity_stats_confirmed=False
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
            deadline = time.monotonic() + (35 if args.command in ("C","}","!","+") else 30 if args.command in ("j", "r", "n", "m", "f", "g", "w", "z", "y", "q", "Q", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "}", "!", "+", "_", "@", "%", ":", ";", "(", ")", "?", "^", "e", "l") else 25)
            while time.monotonic() < deadline:
                for line in read_lines():
                    if args.command in ('@','%',':',';','(',')','?','^') and line.startswith('CURRENT_TRACKING '):
                        verify_current_tracking(line,q_time=.005 if args.command in (':',';','(',')','?','^') else .002);tracking_config_confirmed=True
                    if args.command in ('@','%',':',';','(',')','?','^') and line.startswith('CURRENT_TRACKING_ACTIVE '):
                        verify_current_tracking(line,True,.005 if args.command in (':',';','(',')','?','^') else .002);tracking_active_confirmed=True
                    if args.command in ('@','%',':',';','(',')','?','^') and line.startswith('RUN CURRENT VELOCITY:') and not (tracking_config_confirmed and tracking_active_confirmed):
                        raise RuntimeError('Missing actual current tracking timing; stopping')
                    if args.command in ('}','!','+') and line.startswith('ALIGN_PROFILE '):
                        verify_run_edge_alignment(line);raw_alignment_confirmed=True
                    if args.command in ('!','+') and line.startswith('ESTIMATOR '):
                        if line!=HIGH_ESTIMATOR:raise RuntimeError('High raw observation guard/observer mismatch; stopping')
                        raw_high_estimator_confirmed=True
                    if args.command in ('!','+') and line.startswith('RUN_EDGE_DRIVE '):
                        if line!=(FAST_DRIVE if args.command=='+' else HIGH_DRIVE):raise RuntimeError('High open-loop drive mismatch; stopping')
                        raw_high_drive_confirmed=True
                    if args.command in ('!','+') and line.startswith('REFERENCE_RAMP '):
                        verify_reference_profile(line,6.283185,800000 if args.command=='+' else 2000000);reference_profile_confirmed=True
                    if line.startswith('RUN_EDGE_CONFIG '):
                        if args.command not in ('}','!','+'):raise RuntimeError('Unexpected raw edge capture; stopping')
                        verify_run_edge_config(line);raw_config_confirmed=True
                    if line.startswith('RUN_EDGE_ACTIVE '):
                        verify_run_edge_active(line,args.command in ('!','+'));raw_active_confirmed=True
                    if args.command in ('}','!','+') and line.startswith('CURRENT_CONFIG '):
                        verify_run_edge_observer(line);raw_observer_confirmed=True
                    if args.command in ('}','!','+') and line.startswith('RUN OPEN:'):
                        if line!=(RAW_HIGH_OPEN if args.command in ('!','+') else RAW_OPEN) or not (edge_clock_confirmed and raw_config_confirmed and raw_active_confirmed and raw_observer_confirmed and raw_alignment_confirmed):
                            raise RuntimeError('Raw edge open-loop profile lacks verified settings; stopping')
                        if args.command in ('!','+') and not (raw_high_estimator_confirmed and raw_high_drive_confirmed and reference_profile_confirmed and speed_guard_confirmed and velocity_stats_confirmed):raise RuntimeError('Higher open-loop profile lacks verified independent bounds/statistics; stopping')
                    if line=='RUN_EDGES_END':raw_end=True
                    if line.startswith('EDGE_CLOCK_CHECK '):
                        if args.command not in DWT_CLOCK_COMMANDS:raise RuntimeError('Unexpected cycle-clock check; stopping')
                        verify_edge_clock_check(line);edge_clock_confirmed=True
                    if args.command in DWT_EDGE_COMMANDS and line.startswith('RUN CURRENT VELOCITY:') and not edge_clock_confirmed:
                        raise RuntimeError('Missing disabled cycle-clock check; stopping')
                    if line.startswith('EDGE_ESTIMATOR_CONFIG '):
                        if args.command not in ('[',']','{','_','@','%',':',';','(',')','?','^'):raise RuntimeError('Unexpected edge estimator; stopping')
                        verify_edge_estimator_config(line,args.command in DWT_EDGE_COMMANDS);edge_estimator_confirmed=True
                    if args.command in ('[',']','{','_','@','%',':',';','(',')','?','^') and line.startswith('RUN CURRENT VELOCITY:') and not (edge_estimator_confirmed and edge_mode_confirmed):
                        raise RuntimeError('Missing edge-estimator configuration; stopping')
                    if line.startswith("FIRMWARE diag_version="):
                        firmware_version = int(line.split("=", 1)[1])
                    if line.startswith("REFERENCE_RAMP ") and args.command in ("W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^"):
                        verify_reference_profile(line,candidate_velocity_profile(args.command)[3],candidate_reference_duration(args.command))
                        reference_profile_confirmed=True
                    if line.startswith('GUARD_PEAK_CONFIG '):
                        verify_guard_peak_config(line,candidate_reference_duration(args.command))
                        guard_peak_config_confirmed=True
                    if firmware_version>=97 and line.startswith('RUN CURRENT VELOCITY:') and not guard_peak_config_confirmed:
                        raise RuntimeError('Missing guard-peak observation configuration; stopping')
                    if line.startswith('SPEED_GUARD_CONFIG '):
                        if args.command not in REFERENCE_GUARD_COMMANDS:raise RuntimeError('Unexpected higher-range guard; stopping')
                        verify_speed_guard_config(line);speed_guard_confirmed=True
                    if line.startswith('VELOCITY_STATS_CONFIG '):
                        if args.command not in REFERENCE_GUARD_COMMANDS:raise RuntimeError('Unexpected higher-range statistics; stopping')
                        verify_velocity_stats_config(line,candidate_reference_duration(args.command));velocity_stats_confirmed=True
                    if args.command in REFERENCE_GUARD_COMMANDS and args.command not in ('?','^') and line.startswith('ADAPTIVE_CONFIG '):
                        raise RuntimeError('Higher-range baseline must not apply adaptive correction; stopping')
                    if args.command in HIGH_RANGE_COMMANDS and line.startswith('RUN CURRENT VELOCITY:') and not (speed_guard_confirmed and velocity_stats_confirmed):
                        raise RuntimeError('Missing higher-range guard or interval statistics; stopping')
                    if args.command in ('?','^') and line.startswith('CURRENT_PROBE '):
                        raise RuntimeError('Adaptive comparison must not inject the response probe; stopping')
                    if line.startswith("CURRENT_PROBE ") and args.command in ("T", "U", "(", ")"):
                        verify_response_probe(line,args.command,args.expect_response_hz)
                        response_probe_config_verified=True
                    if line.startswith("ADAPTIVE_CONFIG ") and args.command in ("T", "U", "(", ")"):
                        raise RuntimeError("Adaptive correction must be disabled during response identification; stopping")
                    if line.startswith("ADAPTIVE_CONFIG ") and args.command in ADAPTIVE_COMMANDS:
                        verify_adaptive_profile(line,args.command)
                        adaptive_config_verified=True
                    if line.startswith("TRACE_CONFIG ") and args.command in ("D", "E", "M", "N", "5", "6", "!", "+", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^") and not line.startswith("TRACE_CONFIG period_us=2000 capacity=320 "):
                        raise RuntimeError("Dense trace cadence/capacity mismatch; stopping")
                    if line.startswith(("ESTIMATOR controller=rolling_LS","ESTIMATOR controller=edge_")) and args.command in ("g", "w", "q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^"):
                        verify_rolling_profile(line,args.command)
                        if args.command in ('[',']','{','_','@','%',':',';','(',')','?','^'):edge_mode_confirmed=True
                    if line.startswith("CURRENT_LOOP "):
                        current_lpf = float(re.search(r"current_LPF_s=([0-9.]+)", line)[1])
                        q_p = float(re.search(r"q_P=([0-9.]+)",line)[1])
                        q_i = float(re.search(r"q_I=([0-9.]+)",line)[1])
                        if firmware_version>=78 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^"):
                            cap=float(re.search(r"vector_voltage_limit_V=([0-9.]+)",line)[1])
                            if abs(q_p-.60)>1e-7 or abs(q_i-(400 if args.command in (';','(',')','?','^') else 200))>1e-7 or abs(cap-.5)>1e-7 or abs(current_lpf-candidate_current_lpf(args.command))>1e-7:
                                raise RuntimeError("Candidate current PI/filter/vector bound mismatch; stopping")
                    if line.startswith("DRIVE current_velocity=1 ") and firmware_version>=78 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^"):
                        target=float(re.search(r"speed_target_rad_s=(-?[0-9.]+)",line)[1])
                        limit=float(re.search(r"nominal_iq_limit_A=([0-9.]+)",line)[1])
                        expected_target=candidate_velocity_profile(args.command)[3]
                        if abs(target-expected_target)>1e-7 or abs(limit-.45)>1e-7:
                            raise RuntimeError("Candidate direction/speed/current-setpoint bound mismatch; stopping")
                    if line.startswith("CONTROL "):
                        velocity_lpf = float(re.search(r"velocity_LPF_s=([0-9.]+)", line)[1])
                        velocity_p=float(re.search(r"(?:^| )P=([0-9.]+)",line)[1])
                        velocity_i=float(re.search(r"(?:^| )I=([0-9.]+)",line)[1])
                        if firmware_version>=76 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^"):
                            vp=float(re.search(r"(?:^| )P=([0-9.]+)",line)[1])
                            vi=float(re.search(r"(?:^| )I=([0-9.]+)",line)[1])
                            expected_p,expected_i,_,_=candidate_velocity_profile(args.command)
                            if abs(vp-expected_p)>1e-7 or abs(vi-expected_i)>1e-7:
                                raise RuntimeError("Candidate velocity P/I mismatch; stopping")
                        if firmware_version>=74 and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^") and abs(velocity_lpf-candidate_velocity_profile(args.command)[2])>1e-7:
                            raise RuntimeError("Candidate velocity-filter configuration mismatch; stopping")
                    if line.startswith("ALIGN_PROFILE ") and args.command in ("q", "Q", "C", "H", "L", "D", "E", "I", "R", "B", "S", "M", "N", "A", "Z", "T", "U", "F", "V", "W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^"):
                        expected_cycles=4 if args.command=="C" else 3
                        expected_deadline=20000000 if args.command=="C" else 0
                        if not line.startswith(f"ALIGN_PROFILE cycles={expected_cycles} ") or f"max_alignment_us={expected_deadline}" not in line:
                            raise RuntimeError("Alignment profile mismatch; stopping")
                    if line.startswith("CURRENT_ACTIVE "):
                        verify_active_filters(line, current_lpf, velocity_lpf, q_p if firmware_version>=65 else None, q_i if firmware_version>=65 else None, velocity_p if firmware_version>=81 else None,velocity_i if firmware_version>=81 else None)
                        active_filters_confirmed = True
                        record("HOST verified active current/velocity filter settings")
                    if line.startswith("RUN CURRENT VELOCITY:") and args.command in ("W", "J", "K", "X", "Y", "O", "P", "2", "5", "6", "7", "8", "9", "0", "G", "1", "3", "4", "=", ">", "<", "~", "[", "]", "{", "_", "@", "%", ":", ";", "(", ")", "?", "^") and not reference_profile_confirmed:
                        raise RuntimeError("Ramped-speed profile missing reference ramp; stopping")
                    if firmware_version >= 58 and line.startswith(("RUN CURRENT VELOCITY:", "RUN CURRENT:")) and not active_filters_confirmed:
                        raise RuntimeError("Current control started without active filter metadata; stopping")
                    if line.startswith("RUN CURRENT VELOCITY:") and args.command in ADAPTIVE_COMMANDS and not adaptive_config_verified:
                        raise RuntimeError("Adaptive comparison started without bounded profile metadata; stopping")
                    if line.startswith("RUN CURRENT VELOCITY:") and args.command in ("T", "U", "(", ")") and not response_probe_config_verified:
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
                if stopped and trace_end and off_rows >= 3 and (args.command not in ('}','!','+') or raw_end):
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
