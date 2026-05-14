#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PYTHON = os.environ.get("PYTHON", sys.executable)


COMMANDS = [
    ("mock_nautilus", ROOT, [PYTHON, "example/python/mock_nautilus_server.py"]),
    (
        "visualizer",
        ROOT / "simulate_python",
        [PYTHON, "g1_vs_g1_seiken_visualizer.py"],
    ),
    ("seiken_g1", ROOT / "example/python", [PYTHON, "seiken_g1.py"]),
]


def main():
    processes = []

    try:
        for name, cwd, command in COMMANDS:
            print(f"Starting {name}: {' '.join(command)}")
            process = subprocess.Popen(command, cwd=cwd)
            processes.append(process)
            time.sleep(2)

        print("\nAll started. Press Ctrl+C to stop.")

        while all(process.poll() is None for process in processes):
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)

        time.sleep(1)

        for process in processes:
            if process.poll() is None:
                process.terminate()


if __name__ == "__main__":
    main()
