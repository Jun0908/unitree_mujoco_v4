import argparse
import math
import time

from so101_client import DEFAULT_HOST, DEFAULT_PORT, HOME, SO101Client


def parse_args():
    parser = argparse.ArgumentParser(description="Make the SO-101 wave.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    return parser.parse_args()


def main():
    args = parse_args()
    arm = SO101Client(args.host, args.port)

    print("Sending SO-101 wave motion. Start the viewer first:")
    print("  python3 simulate_python/so101_visualizer.py --manual")

    ready = [0.0, -0.45, 0.95, -0.55, 0.0, 0.75]
    arm.move_to(HOME, duration=0.5)
    arm.move_to(ready, duration=1.0)

    start = time.perf_counter()
    while True:
        t = time.perf_counter() - start
        q = ready.copy()
        q[0] = 0.35 * math.sin(t * 1.2)
        q[4] = 1.15 * math.sin(t * 4.0)
        q[5] = 0.85 + 0.22 * math.sin(t * 2.0)
        arm.send(q)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
