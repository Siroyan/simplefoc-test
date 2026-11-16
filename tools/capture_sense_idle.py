"""Initialize and inspect the ADC observer with gates OFF; sends only i and s."""
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
    with args.output.open("x", encoding="utf-8", buffering=1) as log:
        def record(line):
            log.write(line + "\n")
            if not line.startswith("SENSE,") and not line[0].isdigit():
                print(line, flush=True)

        link = serial.Serial(port=None, baudrate=115200, timeout=.2)
        link.dtr = link.rts = False
        link.port = args.port
        link.open()
        pending = b""

        def lines():
            nonlocal pending
            pending += link.read(max(1, link.in_waiting))
            while b"\n" in pending:
                raw, pending = pending.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    record(line)
                    yield line

        def is_idle(line):
            fields = line.split(",")
            if len(fields) != 8 or not fields[0].isdigit():
                return False
            if fields[1] != "0" or float(fields[7]) != 0:
                raise RuntimeError("Output not OFF during ADC-only capture")
            return True

        try:
            record(f"HOST sense_idle start={datetime.now().astimezone().isoformat()}; sends only i/s; supply=11.1V/0.8A user setting; CV unmeasured")
            idle = 0
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and idle < 3:
                for line in lines():
                    idle += is_idle(line)
            if idle < 3:
                raise RuntimeError("No live output-OFF telemetry; no init sent")
            link.write(b"i")
            link.flush()
            initialized = False
            rows = []
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                for line in lines():
                    is_idle(line)
                    if line.startswith(("SENSE_FAULT", "Rejected:", "ERROR:")):
                        raise RuntimeError(line)
                    if line.startswith("SENSE_INIT "):
                        if line != "SENSE_INIT ready=1 gates_off=1":
                            raise RuntimeError(line)
                        initialized = True
                    if line.startswith("SENSE,"):
                        values = list(map(float, line.split(",")[1:]))
                        if len(values) != 14 or values[-1] != 1 or values[3] > 2000:
                            raise RuntimeError("Invalid ADC/gate snapshot")
                        if rows and (values[1] <= rows[-1][1] or values[2] <= rows[-1][2]):
                            raise RuntimeError("ADC DMA frame counters stopped")
                        rows.append(values)
                if initialized and len(rows) >= 20:
                    record(f"HOST verified: {len(rows)} ADC-only rows, advancing DMA counters, fresh samples and gates OFF")
                    return
            raise RuntimeError("ADC observer initialization/capture timeout")
        finally:
            link.write(b"s")
            link.flush()
            ack = False
            idle = 0
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                for line in lines():
                    ack |= line == "STOP: serial stop"
                    idle += is_idle(line)
            record(f"HOST final_stop_acknowledged={ack} idle_rows={idle}")
            link.close()
            if not ack or idle < 3:
                raise RuntimeError("Final ADC-only stop verification failed")


if __name__ == "__main__":
    main()
