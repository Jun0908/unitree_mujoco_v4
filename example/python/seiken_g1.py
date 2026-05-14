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
    90, 60, 60,
    70, 55, 45, 45, 25, 15, 15,
    90, 60, 55, 55, 35, 20, 20,
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
    WAIST_ROLL = 13
    WAIST_PITCH = 14
    LEFT_SHOULDER_PITCH = 15
    LEFT_SHOULDER_ROLL = 16
    LEFT_SHOULDER_YAW = 17
    LEFT_ELBOW = 18
    LEFT_WRIST_ROLL = 19
    LEFT_WRIST_PITCH = 20
    LEFT_WRIST_YAW = 21
    RIGHT_SHOULDER_PITCH = 22
    RIGHT_SHOULDER_ROLL = 23
    RIGHT_SHOULDER_YAW = 24
    RIGHT_ELBOW = 25
    RIGHT_WRIST_ROLL = 26
    RIGHT_WRIST_PITCH = 27
    RIGHT_WRIST_YAW = 28


class SeikenG1Controller:
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
            print("Received G1 state. Seiken-zuki motion ready.")

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
        cycle = 2.8
        t = elapsed % cycle

        stance = np.zeros(G1_NUM_MOTOR, dtype=float)
        stance[J.WAIST_PITCH] = 0.02
        stance[J.LEFT_SHOULDER_PITCH] = -0.35
        stance[J.LEFT_SHOULDER_ROLL] = 0.08
        stance[J.LEFT_SHOULDER_YAW] = -0.05
        stance[J.LEFT_ELBOW] = 0.20
        stance[J.LEFT_WRIST_ROLL] = -0.08
        stance[J.RIGHT_SHOULDER_PITCH] = -0.32
        stance[J.RIGHT_SHOULDER_ROLL] = -0.08
        stance[J.RIGHT_SHOULDER_YAW] = 0.05
        stance[J.RIGHT_ELBOW] = 0.25
        stance[J.RIGHT_WRIST_ROLL] = 0.15

        guard = stance.copy()
        guard[J.WAIST_YAW] = 0.03
        guard[J.WAIST_PITCH] = 0.06
        guard[J.LEFT_SHOULDER_PITCH] = 0.06
        guard[J.LEFT_SHOULDER_ROLL] = 0.36
        guard[J.LEFT_SHOULDER_YAW] = -0.18
        guard[J.LEFT_ELBOW] = 1.10
        guard[J.LEFT_WRIST_ROLL] = -0.22
        guard[J.LEFT_WRIST_PITCH] = 0.08
        guard[J.LEFT_WRIST_YAW] = 0.06
        guard[J.RIGHT_SHOULDER_PITCH] = 0.10
        guard[J.RIGHT_SHOULDER_ROLL] = -0.22
        guard[J.RIGHT_SHOULDER_YAW] = 0.04
        guard[J.RIGHT_ELBOW] = 1.20
        guard[J.RIGHT_WRIST_ROLL] = 1.00
        guard[J.RIGHT_WRIST_PITCH] = 0.04
        guard[J.RIGHT_WRIST_YAW] = -0.12

        windup_pose = guard.copy()
        windup_pose[J.WAIST_YAW] = 0.26
        windup_pose[J.WAIST_ROLL] = -0.06
        windup_pose[J.WAIST_PITCH] = 0.14
        windup_pose[J.RIGHT_SHOULDER_PITCH] = 0.52
        windup_pose[J.RIGHT_SHOULDER_ROLL] = -0.48
        windup_pose[J.RIGHT_SHOULDER_YAW] = 0.42
        windup_pose[J.RIGHT_ELBOW] = 1.82
        windup_pose[J.RIGHT_WRIST_ROLL] = 1.25
        windup_pose[J.RIGHT_WRIST_PITCH] = 0.20
        windup_pose[J.RIGHT_WRIST_YAW] = -0.25

        strike_pose = guard.copy()
        strike_pose[J.WAIST_YAW] = -0.38
        strike_pose[J.WAIST_ROLL] = 0.03
        strike_pose[J.WAIST_PITCH] = -0.05
        strike_pose[J.RIGHT_SHOULDER_PITCH] = -1.05
        strike_pose[J.RIGHT_SHOULDER_ROLL] = 0.06
        strike_pose[J.RIGHT_SHOULDER_YAW] = 0.98
        strike_pose[J.RIGHT_ELBOW] = 0.00
        strike_pose[J.RIGHT_WRIST_ROLL] = 1.57
        strike_pose[J.RIGHT_WRIST_PITCH] = -0.02
        strike_pose[J.RIGHT_WRIST_YAW] = 0.00
        strike_pose[J.LEFT_SHOULDER_PITCH] = 0.28
        strike_pose[J.LEFT_SHOULDER_ROLL] = 0.55
        strike_pose[J.LEFT_ELBOW] = 1.35

        stance_p = self.phase(t, 0.0, 0.35)
        guard_p = self.phase(t, 0.35, 0.25)
        windup_p = self.phase(t, 0.60, 0.45)
        strike_p = self.phase(t, 1.05, 0.14)
        retract_p = self.phase(t, 1.19, 0.34)
        reset_p = self.phase(t, 1.70, 0.45)

        pose = self.base_q * (1.0 - stance_p) + (self.base_q + stance) * stance_p
        pose = pose * (1.0 - guard_p) + (self.base_q + guard) * guard_p
        pose = pose * (1.0 - windup_p) + (self.base_q + windup_pose) * windup_p
        pose = pose * (1.0 - strike_p) + (self.base_q + strike_pose) * strike_p
        pose = pose * retract_p + (self.base_q + guard) * (1.0 - retract_p)
        pose = pose * (1.0 - reset_p) + self.base_q * reset_p

        for idx in [
            J.WAIST_YAW,
            J.WAIST_ROLL,
            J.WAIST_PITCH,
            J.LEFT_SHOULDER_PITCH,
            J.LEFT_SHOULDER_ROLL,
            J.LEFT_SHOULDER_YAW,
            J.LEFT_ELBOW,
            J.LEFT_WRIST_ROLL,
            J.LEFT_WRIST_PITCH,
            J.LEFT_WRIST_YAW,
            J.RIGHT_SHOULDER_PITCH,
            J.RIGHT_SHOULDER_ROLL,
            J.RIGHT_SHOULDER_YAW,
            J.RIGHT_ELBOW,
            J.RIGHT_WRIST_ROLL,
            J.RIGHT_WRIST_PITCH,
            J.RIGHT_WRIST_YAW,
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
                    f"seiken_debug t={elapsed:5.2f}s "
                    f"r_elbow={self.low_state.motor_state[J.RIGHT_ELBOW].q:+.3f} "
                    f"r_wrist_roll={self.low_state.motor_state[J.RIGHT_WRIST_ROLL].q:+.3f}"
                )

            sleep_time = CONTROL_DT - (time.perf_counter() - step_start)
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

    SeikenG1Controller().run()
