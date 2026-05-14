import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_


DOMAIN_ID = 1
INTERFACE = "lo"
VIEWER_DT = 0.02
LOWSTATE_DT = 0.01
MODE_MACHINE_29DOF = 5
ROBOT_DOF = 29
HIT_DISTANCE = 0.24


class G1VsG1SeikenVisualizer:
    def __init__(self):
        self.model = self._build_model()
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

        self.a_qpos_adrs = []
        self.a_dof_adrs = []
        for actuator_id in range(ROBOT_DOF):
            joint_id = int(self.model.actuator_trnid[actuator_id][0])
            self.a_qpos_adrs.append(int(self.model.jnt_qposadr[joint_id]))
            self.a_dof_adrs.append(int(self.model.jnt_dofadr[joint_id]))

        self.a_base_qadr = int(self.model.jnt_qposadr[self.model.joint("a_floating_base_joint").id])
        self.b_base_qadr = int(self.model.jnt_qposadr[self.model.joint("b_floating_base_joint").id])
        self.a_elbow_body_id = self.model.body("a_right_elbow_link").id
        self.a_fist_body_id = self.model.body("a_right_wrist_yaw_link").id
        self.b_head_site_id = self.model.site("b_head_target").id
        self.b_head_geom_id = self.model.geom("b_head_marker").id
        self.default_head_rgba = self.model.geom_rgba[self.b_head_geom_id].copy()

        self.last_cmd_time = 0.0
        self.last_publish_time = 0.0
        self.last_debug_time = 0.0
        self.hit_active_until = 0.0
        self.hit_count = 0

        self._set_base_poses()
        self._set_opponent_pose()
        mujoco.mj_forward(self.model, self.data)

    def _build_model(self):
        base = mujoco.MjSpec.from_string(
            """
            <mujoco model="g1_vs_g1_seiken">
              <visual>
                <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0 0 0"/>
                <rgba haze="0.15 0.22 0.30 1"/>
                <global azimuth="-145" elevation="-18"/>
              </visual>
              <asset>
                <texture type="skybox" builtin="gradient" rgb1="0.32 0.46 0.62" rgb2="0.05 0.08 0.10" width="512" height="3072"/>
                <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.22 0.28 0.33" rgb2="0.12 0.18 0.22"
                  markrgb="0.85 0.85 0.85" width="300" height="300"/>
                <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="6 6" reflectance="0.15"/>
              </asset>
              <worldbody>
                <light pos="0 0 1.8" dir="0 0 -1" directional="true"/>
                <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
              </worldbody>
            </mujoco>
            """
        )

        frame_a = base.worldbody.add_frame(name="frame_a", pos=[0.0, 0.0, 0.0])
        frame_b = base.worldbody.add_frame(name="frame_b", pos=[0.42, 0.0, 0.0], quat=[0.0, 0.0, 0.0, 1.0])

        spec_a = mujoco.MjSpec.from_file("../unitree_robots/g1/g1_29dof.xml")
        spec_b = mujoco.MjSpec.from_file("../unitree_robots/g1/g1_29dof.xml")
        spec_b.body("torso_link").add_site(name="head_target", pos=[0.10, 0.0, 0.28], size=[0.03])
        spec_b.body("torso_link").add_geom(
            name="head_marker",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=[0.10, 0.0, 0.28],
            size=[0.13, 0.0, 0.0],
            rgba=[0.72, 0.18, 0.18, 1.0],
            contype=1,
            conaffinity=1,
        )

        base.attach(spec_a, prefix="a_", frame=frame_a)
        base.attach(spec_b, prefix="b_", frame=frame_b)
        return base.compile()

    def _set_base_poses(self):
        self.data.qpos[self.a_base_qadr : self.a_base_qadr + 7] = [0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0]
        self.data.qpos[self.b_base_qadr : self.b_base_qadr + 7] = [0.42, 0.0, 0.793, 0.0, 0.0, 0.0, 1.0]

    def _set_opponent_pose(self):
        guard = np.zeros(ROBOT_DOF)
        guard[12] = -0.05
        guard[15] = 0.25
        guard[16] = 0.30
        guard[17] = -0.15
        guard[18] = 1.20
        guard[22] = 0.25
        guard[23] = -0.30
        guard[24] = 0.15
        guard[25] = 1.20
        guard[26] = -0.5
        guard[27] = 0.1
        guard[28] = 0.1

        offset = ROBOT_DOF
        for actuator_id in range(ROBOT_DOF):
            joint_id = int(self.model.actuator_trnid[offset + actuator_id][0])
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            self.data.qpos[qpos_adr] = guard[actuator_id]

    def low_cmd_handler(self, msg: LowCmd_):
        with self.lock:
            for actuator_id in range(ROBOT_DOF):
                self.data.qpos[self.a_qpos_adrs[actuator_id]] = msg.motor_cmd[actuator_id].q
                self.data.qvel[self.a_dof_adrs[actuator_id]] = msg.motor_cmd[actuator_id].dq
            self.low_state.mode_pr = msg.mode_pr
            self.low_state.mode_machine = msg.mode_machine
            mujoco.mj_forward(self.model, self.data)
            self.last_cmd_time = time.perf_counter()
            self.update_hit_state(self.last_cmd_time)

    def update_hit_state(self, now: float):
        elbow_pos = self.data.xpos[self.a_elbow_body_id]
        fist_pos = self.data.xpos[self.a_fist_body_id]
        head_pos = self.data.site_xpos[self.b_head_site_id]
        hand_distance = float(np.linalg.norm(fist_pos - head_pos))
        forearm_distance = self._point_to_segment_distance(head_pos, elbow_pos, fist_pos)
        distance = min(hand_distance, forearm_distance)

        if distance <= HIT_DISTANCE:
            if now > self.hit_active_until:
                self.hit_count += 1
                print(
                    f"CLEAN HIT {self.hit_count}: "
                    f"hand={hand_distance:.3f} m forearm={forearm_distance:.3f} m"
                )
            self.hit_active_until = now + 0.20
            self.model.geom_rgba[self.b_head_geom_id] = [0.15, 0.90, 0.25, 1.0]
        elif now > self.hit_active_until:
            self.model.geom_rgba[self.b_head_geom_id] = self.default_head_rgba

    def _point_to_segment_distance(self, point, seg_a, seg_b):
        segment = seg_b - seg_a
        denom = float(np.dot(segment, segment))
        if denom < 1e-8:
            return float(np.linalg.norm(point - seg_a))
        t = float(np.dot(point - seg_a, segment) / denom)
        t = np.clip(t, 0.0, 1.0)
        closest = seg_a + t * segment
        return float(np.linalg.norm(point - closest))

    def publish_low_state(self):
        for actuator_id in range(ROBOT_DOF):
            self.low_state.motor_state[actuator_id].q = self.data.qpos[self.a_qpos_adrs[actuator_id]]
            self.low_state.motor_state[actuator_id].dq = self.data.qvel[self.a_dof_adrs[actuator_id]]
            self.low_state.motor_state[actuator_id].tau_est = 0.0
        self.low_state_publisher.Write(self.low_state)

    def maybe_print_debug(self, now: float):
        if now - self.last_debug_time < 1.0:
            return
        elbow_pos = self.data.xpos[self.a_elbow_body_id]
        fist_pos = self.data.xpos[self.a_fist_body_id]
        head_pos = self.data.site_xpos[self.b_head_site_id]
        hand_distance = float(np.linalg.norm(fist_pos - head_pos))
        forearm_distance = self._point_to_segment_distance(head_pos, elbow_pos, fist_pos)
        distance = min(hand_distance, forearm_distance)
        print(
            "g1_vs_g1 "
            f"cmd_age={now - self.last_cmd_time:+.3f} "
            f"fist=({fist_pos[0]:+.3f},{fist_pos[1]:+.3f},{fist_pos[2]:+.3f}) "
            f"head=({head_pos[0]:+.3f},{head_pos[1]:+.3f},{head_pos[2]:+.3f}) "
            f"hand={hand_distance:.3f} forearm={forearm_distance:.3f} hit={distance:.3f}"
        )
        self.last_debug_time = now

    def run(self):
        while self.viewer.is_running():
            now = time.perf_counter()
            with self.lock:
                if now - self.last_publish_time >= LOWSTATE_DT:
                    self.publish_low_state()
                    self.last_publish_time = now
                self.update_hit_state(now)
                self.maybe_print_debug(now)
                self.viewer.sync()
            time.sleep(VIEWER_DT)


if __name__ == "__main__":
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)
    G1VsG1SeikenVisualizer().run()
