import argparse
import json
import math
import socket
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer


DEFAULT_MODEL = (
    Path(__file__).resolve().parents[1]
    / "unitree_robots"
    / "so101"
    / "so101_follower.urdf"
)
VIEWER_DT = 0.02
DEFAULT_COMMAND_HOST = "127.0.0.1"
DEFAULT_COMMAND_PORT = 12001


class So101Viewer:
    def __init__(
        self,
        model_path: Path,
        animate: bool,
        speed: float,
        step: float,
        command_host: str,
        command_port: int,
        listen: bool,
    ):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.animate = animate
        self.speed = speed
        self.step = step
        self.command_host = command_host
        self.command_port = command_port
        self.listen = listen
        self.selected_joint = 0
        self.start_time = time.perf_counter()
        self.lock = threading.Lock()
        self.remote_thread = None
        self.remote_socket = None

        self.joint_qpos_adrs = [
            int(self.model.jnt_qposadr[joint_id]) for joint_id in range(self.model.njnt)
        ]
        self.joint_dof_adrs = [
            int(self.model.jnt_dofadr[joint_id]) for joint_id in range(self.model.njnt)
        ]
        self.home_qpos = self._home_qpos()
        self.amplitudes = self._joint_amplitudes()

        self.data.qpos[: self.model.nq] = self.home_qpos
        mujoco.mj_forward(self.model, self.data)

        if self.listen:
            self.start_command_listener()

    def _home_qpos(self):
        home = self.data.qpos.copy()
        for joint_id, qpos_adr in enumerate(self.joint_qpos_adrs):
            low, high = self.model.jnt_range[joint_id]
            name = self.model.joint(joint_id).name
            if name == "gripper":
                home[qpos_adr] = 0.35
            else:
                home[qpos_adr] = 0.5 * (low + high)
        return home

    def _joint_amplitudes(self):
        amplitudes = []
        for joint_id in range(self.model.njnt):
            low, high = self.model.jnt_range[joint_id]
            name = self.model.joint(joint_id).name
            scale = 0.18 if name == "gripper" else 0.22
            amplitudes.append(scale * (high - low))
        return amplitudes

    def clamp_joint(self, joint_id: int):
        qpos_adr = self.joint_qpos_adrs[joint_id]
        low, high = self.model.jnt_range[joint_id]
        self.data.qpos[qpos_adr] = min(max(self.data.qpos[qpos_adr], low), high)

    def move_selected_joint(self, direction: float):
        with self.lock:
            self.animate = False
            qpos_adr = self.joint_qpos_adrs[self.selected_joint]
            dof_adr = self.joint_dof_adrs[self.selected_joint]
            self.data.qpos[qpos_adr] += direction * self.step
            self.data.qvel[dof_adr] = 0.0
            self.clamp_joint(self.selected_joint)
            mujoco.mj_forward(self.model, self.data)
        self.print_selected_joint()

    def print_selected_joint(self):
        qpos_adr = self.joint_qpos_adrs[self.selected_joint]
        joint = self.model.joint(self.selected_joint)
        print(
            f"selected joint[{self.selected_joint + 1}] {joint.name}: "
            f"q={self.data.qpos[qpos_adr]:+.3f} rad"
        )

    def print_controls(self):
        print("Controls:")
        print("  1-6: select joint")
        print("  Q/E: move selected joint")
        print("  Z/C: fine move selected joint")
        print("  Space: pause/resume demo animation")
        print("  R: reset pose")
        print("  H: print controls")
        if self.listen:
            print(
                "Remote control:"
                f"  UDP JSON on {self.command_host}:{self.command_port}"
            )
        self.print_selected_joint()

    def key_callback(self, keycode: int):
        if keycode == ord(" "):
            with self.lock:
                self.animate = not self.animate
        elif keycode in (ord("R"), ord("r")):
            with self.lock:
                self.data.qpos[: self.model.nq] = self.home_qpos
                self.data.qvel[: self.model.nv] = 0.0
                mujoco.mj_forward(self.model, self.data)
                self.start_time = time.perf_counter()
                self.print_selected_joint()
        elif keycode in (ord("H"), ord("h")):
            self.print_controls()
        elif ord("1") <= keycode <= ord("9"):
            joint_id = keycode - ord("1")
            if joint_id < self.model.njnt:
                self.selected_joint = joint_id
                self.print_selected_joint()
        elif keycode in (ord("Q"), ord("q")):
            self.move_selected_joint(-1.0)
        elif keycode in (ord("E"), ord("e")):
            self.move_selected_joint(1.0)
        elif keycode in (ord("Z"), ord("z")):
            self.move_selected_joint(-0.25)
        elif keycode in (ord("C"), ord("c")):
            self.move_selected_joint(0.25)

    def print_model_info(self):
        print(
            "SO-101 model loaded: "
            f"bodies={self.model.nbody}, joints={self.model.njnt}, "
            f"geoms={self.model.ngeom}, meshes={self.model.nmesh}"
        )
        for joint_id in range(self.model.njnt):
            low, high = self.model.jnt_range[joint_id]
            print(
                f"  joint[{joint_id}] {self.model.joint(joint_id).name}: "
                f"range=({low:+.3f}, {high:+.3f}) rad"
            )

    def update_motion(self):
        if not self.animate:
            return

        now = time.perf_counter() - self.start_time
        phase_step = math.tau / max(1, self.model.njnt)
        for joint_id, (qpos_adr, dof_adr) in enumerate(
            zip(self.joint_qpos_adrs, self.joint_dof_adrs)
        ):
            phase = joint_id * phase_step
            frequency = self.speed * (0.35 + 0.04 * joint_id)
            angle = now * frequency + phase
            self.data.qpos[qpos_adr] = (
                self.home_qpos[qpos_adr] + self.amplitudes[joint_id] * math.sin(angle)
            )
            self.data.qvel[dof_adr] = self.amplitudes[joint_id] * frequency * math.cos(
                angle
            )

        mujoco.mj_forward(self.model, self.data)

    def start_command_listener(self):
        self.remote_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_socket.bind((self.command_host, self.command_port))
        self.remote_thread = threading.Thread(
            target=self.command_listener_loop,
            name="so101_command_listener",
            daemon=True,
        )
        self.remote_thread.start()

    def command_listener_loop(self):
        while True:
            packet, _ = self.remote_socket.recvfrom(4096)
            try:
                command = json.loads(packet.decode("utf-8"))
                self.apply_remote_command(command)
            except Exception as exc:
                print(f"ignored SO-101 command: {exc}")

    def apply_remote_command(self, command):
        if command.get("type", "qpos") != "qpos":
            return

        qpos = command.get("q")
        if not isinstance(qpos, list) or len(qpos) != self.model.njnt:
            raise ValueError(f"expected q list with {self.model.njnt} values")

        with self.lock:
            self.animate = False
            for joint_id, value in enumerate(qpos):
                qpos_adr = self.joint_qpos_adrs[joint_id]
                dof_adr = self.joint_dof_adrs[joint_id]
                self.data.qpos[qpos_adr] = float(value)
                self.data.qvel[dof_adr] = 0.0
                self.clamp_joint(joint_id)
            mujoco.mj_forward(self.model, self.data)

    def run(self):
        self.print_model_info()
        self.print_controls()
        with mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self.key_callback
        ) as viewer:
            viewer.cam.lookat[:] = self.model.stat.center
            viewer.cam.distance = 1.4 * self.model.stat.extent
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25

            while viewer.is_running():
                with self.lock:
                    self.update_motion()
                    viewer.sync()
                time.sleep(VIEWER_DT)


def parse_args():
    parser = argparse.ArgumentParser(description="Display the LeRobot SO-101 arm.")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="Path to the SO-101 URDF model.",
    )
    parser.add_argument(
        "--static",
        action="store_true",
        help="Open the viewer without the demo joint animation.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Animation speed multiplier.",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.05,
        help="Manual joint movement step in radians.",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Start with the demo animation paused for manual control.",
    )
    parser.add_argument(
        "--command-host",
        default=DEFAULT_COMMAND_HOST,
        help="Host/interface for UDP commands from example/python scripts.",
    )
    parser.add_argument(
        "--command-port",
        type=int,
        default=DEFAULT_COMMAND_PORT,
        help="UDP port for commands from example/python scripts.",
    )
    parser.add_argument(
        "--no-listen",
        action="store_true",
        help="Disable the UDP command listener.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Load the model and print information without opening the viewer.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    viewer = So101Viewer(
        args.model,
        animate=not args.static and not args.manual,
        speed=args.speed,
        step=args.step,
        command_host=args.command_host,
        command_port=args.command_port,
        listen=not args.no_listen,
    )
    if args.check:
        viewer.print_model_info()
        return
    viewer.run()


if __name__ == "__main__":
    main()
