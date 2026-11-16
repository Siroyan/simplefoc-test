"""Record a disabled, drive-locked hand-turn session; never sends t/o or resets."""
import argparse
from datetime import datetime
from pathlib import Path
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    args = parser.parse_args()
    if args.stop_file.exists():
        raise RuntimeError("Stop file already exists; choose a new session name")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8", buffering=1) as log:
        last_display = 0

        def record(line):
            nonlocal last_display
            log.write(line + "\n")
            # Retain every row on disk; keep the terminal readable.
            if not line.startswith("MANUAL,") or time.monotonic() - last_display >= 5:
                print(line, flush=True)
                if line.startswith("MANUAL,"):
                    last_display = time.monotonic()

        link = serial.Serial(port=None, baudrate=115200, timeout=0.2)
        link.dtr = link.rts = False
        link.port = args.port
        link.open()
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

        def wait_for(prefix, timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                for line in read_lines():
                    if line.startswith(prefix):
                        return
            raise RuntimeError(f"No acknowledgement: {prefix}")

        try:
            record(f"HOST manual start={datetime.now().astimezone().isoformat()} port={args.port}")
            record("HOST hand-turn only; no drive commands; timeout=600s")
            link.reset_input_buffer()
            link.write(b"s")
            link.flush()
            wait_for("STOP: serial stop", 5)
            link.write(b"h")
            link.flush()
            wait_for("MANUAL_BEGIN:", 5)
            # This marker is printed only after the board confirms drive lock.
            record("HOST RECORDING_READY: hand-turn may begin; create stop file when finished")
            deadline = time.monotonic() + 600
            ended = False
            while time.monotonic() < deadline and not args.stop_file.exists():
                for line in read_lines():
                    if line.startswith("MANUAL_END:"):
                        ended = True
                    if line.startswith(("READY:", "ERROR:", "RUN:", "RUN OPEN:")):
                        raise RuntimeError("Unexpected firmware state during manual capture")
                if ended:
                    raise RuntimeError("Manual capture ended unexpectedly")
            record("HOST ending capture: stop file or 600s timeout")
            link.write(b"x")
            link.flush()
            wait_for("MANUAL_END:", 8)
        finally:
            if link.is_open:
                link.write(b"s")
                link.flush()
                try:
                    wait_for("STOP: serial stop", 5)
                    record("HOST final_stop_acknowledged=True")
                finally:
                    link.close()


if __name__ == "__main__":
    main()
