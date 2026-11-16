"""Disable output, lock drive commands via manual mode, and verify idle telemetry."""
import argparse
from datetime import datetime
from pathlib import Path
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8", buffering=1) as log:
        def record(line):
            print(line, flush=True)
            log.write(line + "\n")

        link = serial.Serial(port=None, baudrate=115200, timeout=0.2)
        link.dtr = link.rts = False
        link.port = args.port
        link.open()
        buffer = b""
        acknowledged = locked = False
        idle_rows = 0
        try:
            record(f"HOST lock start={datetime.now().astimezone().isoformat()}; sends only s/h/x")
            link.reset_input_buffer()
            link.write(b"shx")
            link.flush()
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                buffer += link.read(max(1, link.in_waiting))
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    record(line)
                    acknowledged |= line == "STOP: serial stop"
                    locked |= line == "MANUAL_END: gates OFF; drive locked until reset."
                    fields = line.split(",")
                    if locked and len(fields) == 8 and fields[0].isdigit():
                        if fields[1] != "0" or float(fields[7]) != 0:
                            raise RuntimeError("Idle output verification failed")
                        idle_rows += 1
                if acknowledged and locked and idle_rows >= 5:
                    record("HOST verified: STOP acknowledged, drive locked until reset, five output-OFF rows")
                    return
            raise RuntimeError("No complete output-OFF/drive-lock acknowledgement")
        finally:
            link.write(b"s")
            link.flush()
            link.close()


if __name__ == "__main__":
    main()
