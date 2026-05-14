import argparse

from so101_client import DEFAULT_HOST, DEFAULT_PORT, HOME, SO101Client


def parse_args():
    parser = argparse.ArgumentParser(description="Run a simple SO-101 pick/place pose sequence.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    return parser.parse_args()


def main():
    args = parse_args()
    arm = SO101Client(args.host, args.port)

    print("Sending SO-101 pick/place motion. Start the viewer first:")
    print("  python3 simulate_python/so101_visualizer.py --manual")

    ready = [0.0, -0.45, 0.85, -0.50, 0.0, 1.05]
    approach_pick = [0.48, -0.88, 1.22, -0.82, 0.0, 1.20]
    grasp = [0.48, -0.88, 1.22, -0.82, 0.0, 0.12]
    lift = [0.30, -0.38, 0.82, -0.58, 0.0, 0.12]
    approach_place = [-0.58, -0.70, 1.02, -0.70, 0.0, 0.12]
    release = [-0.58, -0.70, 1.02, -0.70, 0.0, 1.20]

    while True:
        arm.move_to(HOME, duration=0.7)
        arm.move_to(ready, duration=0.8)
        arm.move_to(approach_pick, duration=0.9)
        arm.hold(0.25)
        arm.move_to(grasp, duration=0.35)
        arm.hold(0.35)
        arm.move_to(lift, duration=0.8)
        arm.move_to(approach_place, duration=1.0)
        arm.hold(0.25)
        arm.move_to(release, duration=0.35)
        arm.hold(0.45)
        arm.move_to(ready, duration=0.8)


if __name__ == "__main__":
    main()
