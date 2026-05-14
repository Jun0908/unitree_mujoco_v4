import sys
import time

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


G1_NUM_MOTOR = 29
CONTROL_DT = 0.01
DOMAIN_ID = 1
INTERFACE = "lo"

KP = [
    80, 60, 60, 120, 40, 40,
    80, 60, 60, 120, 40, 40,
    80, 50, 50,
    60, 50, 40, 40, 20, 20, 20,
    60, 50, 40, 40, 20, 20, 20,
]

KD = [
    2, 1, 1, 3, 1, 1,
    2, 1, 1, 3, 1, 1,
    2, 1, 1,
    2, 1, 1, 1, 1, 1, 1,
    2, 1, 1, 1, 1, 1, 1,
]


class J:
    WAIST_YAW = 12
    LEFT_SHOULDER_PITCH = 15
    LEFT_SHOULDER_ROLL = 16
    LEFT_SHOULDER_YAW = 17
    LEFT_ELBOW = 18
    RIGHT_SHOULDER_PITCH = 22
    RIGHT_SHOULDER_ROLL = 23
    RIGHT_SHOULDER_YAW = 24
    RIGHT_ELBOW = 25


class PunchG1Controller:
    def __init__(self):
        self.low_state = None
        self.base_q = np.zeros(G1_NUM_MOTOR, dtype=float)
        self.have_state = False
        self.mode_machine = 5
        self.crc = CRC()
        self.start_time = None

        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.low_state_handler, 10)

        self.cmd = unitree_hg_msg_dds__LowCmd_()

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg
        if not self.have_state:
            for i in range(G1_NUM_MOTOR):
                self.base_q[i] = msg.motor_state[i].q
            self.mode_machine = msg.mode_machine
            self.have_state = True
            print("Received G1 state. Punch motion ready.")

    def initialize_command(self):
        self.cmd.mode_pr = 0
        self.cmd.mode_machine = self.mode_machine
        for i in range(G1_NUM_MOTOR):
            self.cmd.motor_cmd[i].mode = 1
            self.cmd.motor_cmd[i].q = self.base_q[i]
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kp = KP[i]
            self.cmd.motor_cmd[i].kd = KD[i]
            self.cmd.motor_cmd[i].tau = 0.0

    def smoothstep(self, x: float) -> float:
        x = np.clip(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)

    def phase(self, t: float, start: float, duration: float) -> float:
        return self.smoothstep((t - start) / duration)

    def update_motion(self, elapsed: float):
        cycle = 2.4
        t = elapsed % cycle

        guard = np.zeros(G1_NUM_MOTOR, dtype=float)
        guard[J.WAIST_YAW] = 0.10
        guard[J.LEFT_SHOULDER_PITCH] = 0.15
        guard[J.LEFT_SHOULDER_ROLL] = 0.35
        guard[J.LEFT_SHOULDER_YAW] = -0.20
        guard[J.LEFT_ELBOW] = 1.10
        guard[J.RIGHT_SHOULDER_PITCH] = 0.30
        guard[J.RIGHT_SHOULDER_ROLL] = -0.10
        guard[J.RIGHT_SHOULDER_YAW] = 0.15
        guard[J.RIGHT_ELBOW] = 1.40

        windup = self.phase(t, 0.0, 0.45)
        punch = self.phase(t, 0.45, 0.22)
        retract = self.phase(t, 0.67, 0.35)
        settle = self.phase(t, 1.20, 0.40)

        punch_pose = guard.copy()
        punch_pose[J.WAIST_YAW] = -0.35
        punch_pose[J.RIGHT_SHOULDER_PITCH] = -0.55
        punch_pose[J.RIGHT_SHOULDER_ROLL] = -0.28
        punch_pose[J.RIGHT_SHOULDER_YAW] = 0.55
        punch_pose[J.RIGHT_ELBOW] = 0.20
        punch_pose[J.LEFT_SHOULDER_PITCH] = 0.25
        punch_pose[J.LEFT_SHOULDER_ROLL] = 0.50
        punch_pose[J.LEFT_ELBOW] = 1.25

        pose = self.base_q + guard
        pose = pose + windup * np.array([
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0.18, 0, 0,
            0.08, -0.08, -0.12, -0.18, 0, 0, 0,
            0.12, 0.10, 0.25, 0.30, 0, 0, 0,
        ])
        pose = pose * (1.0 - punch) + (self.base_q + punch_pose) * punch
        pose = pose * retract + (self.base_q + guard) * (1.0 - retract)
        pose = pose * (1.0 - settle) + self.base_q * settle

        for idx in [
            J.WAIST_YAW,
            J.LEFT_SHOULDER_PITCH,
            J.LEFT_SHOULDER_ROLL,
            J.LEFT_SHOULDER_YAW,
            J.LEFT_ELBOW,
            J.RIGHT_SHOULDER_PITCH,
            J.RIGHT_SHOULDER_ROLL,
            J.RIGHT_SHOULDER_YAW,
            J.RIGHT_ELBOW,
        ]:
            self.cmd.motor_cmd[idx].q = pose[idx]

    def run(self):
        print("Waiting for rt/lowstate from the visualizer...")
        while not self.have_state:
            time.sleep(0.1)

        self.start_time = time.perf_counter()
        while True:
            step_start = time.perf_counter()
            elapsed = step_start - self.start_time

            self.initialize_command()
            self.update_motion(elapsed)
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.publisher.Write(self.cmd)

            if self.low_state is not None and int(elapsed) != int(max(elapsed - CONTROL_DT, 0)):
                print(
                    f"punch_debug t={elapsed:5.2f}s "
                    f"right_elbow={self.low_state.motor_state[J.RIGHT_ELBOW].q:+.3f} "
                    f"right_shoulder_pitch={self.low_state.motor_state[J.RIGHT_SHOULDER_PITCH].q:+.3f}"
                )

            sleep_time = CONTROL_DT - (time.perf_counter() - step_start)
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

    PunchG1Controller().run()
