import json
import socket
import time


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 12001
CONTROL_DT = 0.02

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

JOINT_LIMITS = [
    (-1.91986, 1.91986),
    (-1.74533, 1.74533),
    (-1.69000, 1.69000),
    (-1.65806, 1.65806),
    (-2.74385, 2.84121),
    (-0.174533, 1.74533),
]

HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.35]


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def clamp_pose(q):
    return [
        min(max(float(value), low), high)
        for value, (low, high) in zip(q, JOINT_LIMITS)
    ]


def lerp_pose(a, b, phase: float):
    return [ai * (1.0 - phase) + bi * phase for ai, bi in zip(a, b)]


class SO101Client:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.current_q = HOME.copy()

    def send(self, q):
        self.current_q = clamp_pose(q)
        payload = json.dumps({"type": "qpos", "q": self.current_q}).encode("utf-8")
        self.socket.sendto(payload, (self.host, self.port))

    def move_to(self, target_q, duration: float = 1.0, rate_hz: float = 50.0):
        target_q = clamp_pose(target_q)
        start_q = self.current_q.copy()
        steps = max(1, int(duration * rate_hz))
        dt = 1.0 / rate_hz

        for step in range(steps + 1):
            phase = smoothstep(step / steps)
            self.send(lerp_pose(start_q, target_q, phase))
            time.sleep(dt)

    def hold(self, duration: float, rate_hz: float = 20.0):
        steps = max(1, int(duration * rate_hz))
        dt = 1.0 / rate_hz
        for _ in range(steps):
            self.send(self.current_q)
            time.sleep(dt)


def print_joint_order():
    print("SO-101 joint order:")
    for index, name in enumerate(JOINT_NAMES):
        low, high = JOINT_LIMITS[index]
        print(f"  {index}: {name:14s} range=({low:+.3f}, {high:+.3f}) rad")
