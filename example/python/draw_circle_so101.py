import argparse
import math
import time

from so101_client import DEFAULT_HOST, DEFAULT_PORT, HOME, SO101Client


def parse_args():
    parser = argparse.ArgumentParser(description="Move the SO-101 through a looping draw motion.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    return parser.parse_args()


def main():
    args = parse_args()
    arm = SO101Client(args.host, args.port)

    print("Sending SO-101 draw-circle motion. Start the viewer first:")
    print("  python3 simulate_python/so101_visualizer.py --manual")

    center = [0.0, -0.62, 1.05, -0.82, 0.0, 0.22]
    arm.move_to(HOME, duration=0.5)
    arm.move_to(center, duration=1.0)

    start = time.perf_counter()
    while True:
        t = time.perf_counter() - start
        q = center.copy()
        q[0] += 0.42 * math.sin(t * 1.4)
        q[1] += 0.22 * math.cos(t * 1.4)
        q[2] += 0.24 * math.sin(t * 1.4 + math.pi * 0.45)
        q[3] += 0.28 * math.cos(t * 1.4 + math.pi * 0.35)
        q[4] = 0.35 * math.sin(t * 2.8)
        arm.send(q)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
