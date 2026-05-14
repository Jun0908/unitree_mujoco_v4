import threading
import time

import mujoco
import mujoco.viewer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_


ROBOT_SCENE = "../unitree_robots/g1/scene.xml"
DOMAIN_ID = 1
INTERFACE = "lo"
VIEWER_DT = 0.02
LOWSTATE_DT = 0.01
MODE_MACHINE_29DOF = 5


class G1Visualizer:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(ROBOT_SCENE)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.lock = threading.Lock()

        self.low_state = unitree_hg_msg_dds__LowState_()
        self.low_state.mode_pr = 0
        self.low_state.mode_machine = MODE_MACHINE_29DOF
        self.low_state_publisher = ChannelPublisher("rt/lowstate", LowState_)
        self.low_state_publisher.Init()

        self.low_cmd_subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.low_cmd_subscriber.Init(self.low_cmd_handler, 10)

        self.actuator_joint_ids = []
        self.actuator_qpos_adrs = []
        self.actuator_dof_adrs = []
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id][0])
            self.actuator_joint_ids.append(joint_id)
            self.actuator_qpos_adrs.append(int(self.model.jnt_qposadr[joint_id]))
            self.actuator_dof_adrs.append(int(self.model.jnt_dofadr[joint_id]))

        self.last_cmd_time = 0.0
        self.last_publish_time = 0.0
        self.last_debug_time = 0.0

        mujoco.mj_forward(self.model, self.data)

    def low_cmd_handler(self, msg: LowCmd_):
        with self.lock:
            for actuator_id in range(self.model.nu):
                qpos_adr = self.actuator_qpos_adrs[actuator_id]
                dof_adr = self.actuator_dof_adrs[actuator_id]
                self.data.qpos[qpos_adr] = msg.motor_cmd[actuator_id].q
                self.data.qvel[dof_adr] = msg.motor_cmd[actuator_id].dq

            self.low_state.mode_pr = msg.mode_pr
            self.low_state.mode_machine = msg.mode_machine
            mujoco.mj_forward(self.model, self.data)
            self.last_cmd_time = time.perf_counter()

    def publish_low_state(self):
        for actuator_id in range(self.model.nu):
            self.low_state.motor_state[actuator_id].q = self.data.sensordata[actuator_id]
            self.low_state.motor_state[actuator_id].dq = self.data.sensordata[
                actuator_id + self.model.nu
            ]
            self.low_state.motor_state[actuator_id].tau_est = 0.0

        self.low_state_publisher.Write(self.low_state)

    def maybe_print_debug(self, now: float):
        if now - self.last_debug_time < 1.0:
            return

        joint_id = 16
        print(
            "viz_debug "
            f"cmd_age={now - self.last_cmd_time:+.3f} "
            f"q16={self.data.sensordata[joint_id]:+.3f} "
            f"dq16={self.data.sensordata[joint_id + self.model.nu]:+.3f}"
        )
        self.last_debug_time = now

    def run(self):
        while self.viewer.is_running():
            now = time.perf_counter()
            with self.lock:
                if now - self.last_publish_time >= LOWSTATE_DT:
                    self.publish_low_state()
                    self.last_publish_time = now
                self.maybe_print_debug(now)
                self.viewer.sync()
            time.sleep(VIEWER_DT)


if __name__ == "__main__":
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)
    visualizer = G1Visualizer()
    visualizer.run()
