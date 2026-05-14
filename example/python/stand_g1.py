import sys
import time
import json

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29
CONTROL_DT = 0.002
DOMAIN_ID = 1
INTERFACE = "lo"
LOCAL_CMD_PATH = "/tmp/unitree_mujoco_lowcmd.json"

KP = [
    80, 60, 60, 120, 40, 40,
    80, 60, 60, 120, 40, 40,
    60, 40, 40,
    30, 30, 30, 30, 20, 20, 20,
    30, 30, 30, 30, 20, 20, 20,
]

KD = [
    2, 1, 1, 3, 1, 1,
    2, 1, 1, 3, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]


class G1JointIndex:
    WAIST_YAW = 12
    LEFT_SHOULDER_PITCH = 15
    LEFT_SHOULDER_ROLL = 16
    LEFT_ELBOW = 18
    RIGHT_SHOULDER_PITCH = 22
    RIGHT_SHOULDER_ROLL = 23
    RIGHT_ELBOW = 25


class Mode:
    PR = 0


class G1DemoController:
    def __init__(self):
        self.low_state = None
        self.mode_machine = 0
        self.have_state = False
        self.crc = CRC()
        self.start_time = None
        self.last_debug_time = 0.0
        self.last_local_write_time = 0.0

        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.low_state_handler, 10)

        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.base_q = np.zeros(G1_NUM_MOTOR, dtype=float)

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg
        if not self.have_state:
            for i in range(G1_NUM_MOTOR):
                self.base_q[i] = msg.motor_state[i].q
            self.mode_machine = msg.mode_machine
            self.have_state = True
            print("Received G1 state. Starting motion.")

    def initialize_command(self):
        self.cmd.mode_pr = Mode.PR
        self.cmd.mode_machine = self.mode_machine
        for i in range(G1_NUM_MOTOR):
            self.cmd.motor_cmd[i].mode = 1
            self.cmd.motor_cmd[i].q = self.base_q[i]
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kp = KP[i]
            self.cmd.motor_cmd[i].kd = KD[i]
            self.cmd.motor_cmd[i].tau = 0.0

    def update_motion(self, elapsed: float):
        waist_yaw = 0.30 * np.sin(2.0 * np.pi * 0.20 * elapsed)
        shoulder_roll = 0.45 * np.sin(2.0 * np.pi * 0.35 * elapsed)
        shoulder_pitch = 0.35 * np.sin(2.0 * np.pi * 0.20 * elapsed)
        elbow = 0.55 + 0.25 * np.sin(2.0 * np.pi * 0.35 * elapsed + np.pi)

        self.cmd.motor_cmd[G1JointIndex.WAIST_YAW].q = (
            self.base_q[G1JointIndex.WAIST_YAW] + waist_yaw
        )

        self.cmd.motor_cmd[G1JointIndex.LEFT_SHOULDER_ROLL].q = (
            self.base_q[G1JointIndex.LEFT_SHOULDER_ROLL] + shoulder_roll
        )
        self.cmd.motor_cmd[G1JointIndex.RIGHT_SHOULDER_ROLL].q = (
            self.base_q[G1JointIndex.RIGHT_SHOULDER_ROLL] - shoulder_roll
        )
        self.cmd.motor_cmd[G1JointIndex.LEFT_SHOULDER_PITCH].q = (
            self.base_q[G1JointIndex.LEFT_SHOULDER_PITCH] + shoulder_pitch
        )
        self.cmd.motor_cmd[G1JointIndex.RIGHT_SHOULDER_PITCH].q = (
            self.base_q[G1JointIndex.RIGHT_SHOULDER_PITCH] + shoulder_pitch
        )
        self.cmd.motor_cmd[G1JointIndex.LEFT_ELBOW].q = (
            self.base_q[G1JointIndex.LEFT_ELBOW] + elbow
        )
        self.cmd.motor_cmd[G1JointIndex.RIGHT_ELBOW].q = (
            self.base_q[G1JointIndex.RIGHT_ELBOW] - elbow
        )

    def run(self):
        print("Waiting for rt/lowstate from the simulator...")
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
            self.write_local_command(elapsed)

            if self.low_state is not None and elapsed - self.last_debug_time >= 1.0:
                joint_id = G1JointIndex.LEFT_SHOULDER_ROLL
                target_q = self.cmd.motor_cmd[joint_id].q
                actual_q = self.low_state.motor_state[joint_id].q
                actual_dq = self.low_state.motor_state[joint_id].dq
                print(
                    f"debug t={elapsed:5.2f}s "
                    f"target_q={target_q:+.3f} "
                    f"actual_q={actual_q:+.3f} "
                    f"actual_dq={actual_dq:+.3f}"
                )
                self.last_debug_time = elapsed

            sleep_time = CONTROL_DT - (time.perf_counter() - step_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def write_local_command(self, elapsed: float):
        if elapsed - self.last_local_write_time < 0.02:
            return

        payload = {
            "motor_cmd": [
                {
                    "mode": self.cmd.motor_cmd[i].mode,
                    "q": self.cmd.motor_cmd[i].q,
                    "dq": self.cmd.motor_cmd[i].dq,
                    "kp": self.cmd.motor_cmd[i].kp,
                    "kd": self.cmd.motor_cmd[i].kd,
                    "tau": self.cmd.motor_cmd[i].tau,
                }
                for i in range(G1_NUM_MOTOR)
            ]
        }
        try:
            with open(LOCAL_CMD_PATH, "w", encoding="utf-8") as f:
                json.dump(payload, f)
        except OSError:
            return
        self.last_local_write_time = elapsed


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

    controller = G1DemoController()
    controller.run()
