from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

import cv2
import mujoco
import numpy as np

from .poses import ARM_POSES
from .scene_builder import ARM_JOINT_NAMES, ARM_PREFIX


BLOCK_COLORS = ("red", "blue", "yellow")
BLOCK_BODY_BY_COLOR = {color: f"block_{color}" for color in BLOCK_COLORS}
DETECTION_BGR = {
    "red": (40, 40, 230),
    "blue": (230, 120, 30),
    "yellow": (30, 210, 250),
}
HSV_COLOR_RANGES = {
    "red": (
        ((0, 120, 60), (10, 255, 255)),
        ((170, 120, 60), (179, 255, 255)),
    ),
    "blue": (((95, 100, 60), (135, 255, 255)),),
    "yellow": (((18, 90, 80), (40, 255, 255)),),
}


class DemoState(Enum):
    SPAWN = auto()
    CONVEY = auto()
    TRACK = auto()
    STOP_FOR_PICK = auto()
    MOVE_TO_PICK = auto()
    GRASP = auto()
    LIFT = auto()
    MOVE_TO_PLACE = auto()
    RELEASE = auto()
    RETURN_HOME = auto()
    RESUME = auto()


@dataclass(slots=True)
class DemoConfig:
    control_dt: float = 0.01
    camera_name: str = "sorting_cam"
    camera_width: int = 640
    camera_height: int = 480
    capture_camera: bool = True
    camera_stride: int = 2
    conveyor_speed: float = 0.28
    spawn_sequence: tuple[str, ...] = ("red", "blue", "yellow", "red", "yellow", "blue")
    vision_roi: tuple[float, float, float, float] = (0.02, 0.36, 0.98, 0.74)
    foreground_threshold: int = 22
    color_min_area: float = 120.0
    color_max_area: float = 3000.0
    pick_x_tolerance: float = 0.010
    stop_pause_duration: float = 0.03
    resume_pause_duration: float = 0.02
    attached_block_offset: tuple[float, float, float] = (0.0, 0.0, 0.012)
    motion_duration_scale: float = 0.6


@dataclass(slots=True)
class ColorDetection:
    label: str
    bbox: tuple[int, int, int, int]
    center: tuple[int, int]
    area: float

    def summary(self) -> str:
        center_x, center_y = self.center
        return f"{self.label}@({center_x},{center_y}) area={self.area:.0f}"


@dataclass(slots=True)
class MotionSegment:
    name: str
    target_q: np.ndarray
    duration: float


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def lerp_pose(a: np.ndarray, b: np.ndarray, phase: float) -> np.ndarray:
    return a * (1.0 - phase) + b * phase


def quat_from_matrix(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (matrix[2, 1] - matrix[1, 2]) * s
        qy = (matrix[0, 2] - matrix[2, 0]) * s
        qz = (matrix[1, 0] - matrix[0, 1]) * s
    else:
        diagonal = np.diag(matrix)
        axis = int(np.argmax(diagonal))
        if axis == 0:
            s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            qw = (matrix[2, 1] - matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (matrix[0, 1] + matrix[1, 0]) / s
            qz = (matrix[0, 2] + matrix[2, 0]) / s
        elif axis == 1:
            s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            qw = (matrix[0, 2] - matrix[2, 0]) / s
            qx = (matrix[0, 1] + matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            qw = (matrix[1, 0] - matrix[0, 1]) / s
            qx = (matrix[0, 2] + matrix[2, 0]) / s
            qy = (matrix[1, 2] + matrix[2, 1]) / s
            qz = 0.25 * s
    quat = np.array([qw, qx, qy, qz], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat


class ConveyorManager:
    """Move one color block at a time across the conveyor with scripted poses."""

    def __init__(self, model: mujoco.MjModel, config: DemoConfig):
        self.model = model
        self.config = config
        self.sequence = tuple(config.spawn_sequence)
        self.sequence_index = 0
        self.spawn_count = 0
        self.completed_blocks = 0
        self.initialized = False
        self.motion_enabled = True
        self.attached = False
        self.active_color: str | None = None
        self.active_spawn_id: int | None = None
        self.active_position: np.ndarray | None = None
        self.active_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.attached_offset = np.array(config.attached_block_offset, dtype=np.float64)
        self.spawn_site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "spawn_site"))
        self.pick_site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "pick_site"))
        self.drop_site_ids = {
            color: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{color}_drop_site"))
            for color in BLOCK_COLORS
        }
        self.block_body_ids = {
            color: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
            for color, body_name in BLOCK_BODY_BY_COLOR.items()
        }
        self.freejoint_ids = {
            color: int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{BLOCK_BODY_BY_COLOR[color]}_free")
            )
            for color in BLOCK_COLORS
        }
        self.qpos_adrs = {
            color: int(model.jnt_qposadr[joint_id])
            for color, joint_id in self.freejoint_ids.items()
        }
        self.qvel_adrs = {
            color: int(model.jnt_dofadr[joint_id])
            for color, joint_id in self.freejoint_ids.items()
        }
        self.spawn_position = np.zeros(3, dtype=np.float64)
        self.pick_position = np.zeros(3, dtype=np.float64)
        self.exit_x = 0.0
        self.park_positions = {
            color: np.array([-1.30, -0.50 + 0.14 * index, 0.10], dtype=np.float64)
            for index, color in enumerate(BLOCK_COLORS)
        }
        self.stored_positions: dict[str, np.ndarray | None] = {
            color: None for color in BLOCK_COLORS
        }

    def initialize(self, data: mujoco.MjData) -> None:
        self.spawn_position = np.array(data.site_xpos[self.spawn_site_id], dtype=np.float64)
        self.pick_position = np.array(data.site_xpos[self.pick_site_id], dtype=np.float64)
        self.exit_x = float(self.pick_position[0] + 0.24)
        self.hide_all_blocks(data)
        self.initialized = True

    def hide_all_blocks(self, data: mujoco.MjData) -> None:
        for color in BLOCK_COLORS:
            self._set_block_pose(color, data, self.park_positions[color])
            self.stored_positions[color] = None
        self.attached = False
        self.active_color = None
        self.active_spawn_id = None
        self.active_position = None
        self.active_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def pause(self) -> None:
        self.motion_enabled = False

    def resume(self) -> None:
        self.motion_enabled = True

    def _set_block_pose(
        self,
        color: str,
        data: mujoco.MjData,
        position: np.ndarray,
        quat: np.ndarray | None = None,
    ) -> None:
        qpos_adr = self.qpos_adrs[color]
        qvel_adr = self.qvel_adrs[color]
        data.qpos[qpos_adr : qpos_adr + 3] = position
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = (
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            if quat is None
            else quat
        )
        data.qvel[qvel_adr : qvel_adr + 6] = 0.0

    def _spawn_next(self, data: mujoco.MjData) -> None:
        next_color = self.sequence[self.sequence_index % len(self.sequence)]
        self.sequence_index += 1
        self.spawn_count += 1
        self.stored_positions[next_color] = None
        self.active_color = next_color
        self.active_spawn_id = self.spawn_count
        self.active_position = self.spawn_position.copy()
        self.active_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._set_block_pose(next_color, data, self.active_position)

    def _attached_pose(
        self,
        tcp_position: np.ndarray,
        tcp_rotation: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        position = np.array(tcp_position, dtype=np.float64) + tcp_rotation @ self.attached_offset
        quat = quat_from_matrix(tcp_rotation)
        return position, quat

    def attach_active_block(
        self,
        data: mujoco.MjData,
        tcp_position: np.ndarray,
        tcp_rotation: np.ndarray,
    ) -> bool:
        if self.active_color is None or self.active_position is None:
            return False
        self.attached = True
        self.motion_enabled = False
        self.active_position, self.active_quat = self._attached_pose(tcp_position, tcp_rotation)
        self._set_block_pose(self.active_color, data, self.active_position, self.active_quat)
        return True

    def update_attached_block(
        self,
        data: mujoco.MjData,
        tcp_position: np.ndarray,
        tcp_rotation: np.ndarray,
    ) -> None:
        if not self.attached or self.active_color is None:
            return
        self.active_position, self.active_quat = self._attached_pose(tcp_position, tcp_rotation)
        self._set_block_pose(self.active_color, data, self.active_position, self.active_quat)

    def store_active_block(self, data: mujoco.MjData) -> bool:
        if self.active_color is None:
            return False
        drop_site_id = self.drop_site_ids[self.active_color]
        drop_position = np.array(data.site_xpos[drop_site_id], dtype=np.float64)
        self.stored_positions[self.active_color] = drop_position.copy()
        self._set_block_pose(self.active_color, data, drop_position)
        self.completed_blocks += 1
        self.attached = False
        self.active_color = None
        self.active_spawn_id = None
        self.active_position = None
        self.active_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return True

    def update(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if not self.initialized:
            self.initialize(data)

        for color in BLOCK_COLORS:
            if color != self.active_color:
                stored = self.stored_positions[color]
                self._set_block_pose(color, data, stored if stored is not None else self.park_positions[color])

        if self.active_color is None or self.active_position is None:
            if self.motion_enabled:
                self._spawn_next(data)
            return

        if self.attached:
            self._set_block_pose(self.active_color, data, self.active_position, self.active_quat)
            return

        if not self.motion_enabled:
            self._set_block_pose(self.active_color, data, self.active_position)
            return

        self.active_position[0] += self.config.conveyor_speed * float(model.opt.timestep)
        if self.active_position[0] > self.exit_x:
            self.completed_blocks += 1
            self._set_block_pose(self.active_color, data, self.park_positions[self.active_color])
            self.active_color = None
            self.active_spawn_id = None
            self.active_position = None
            self._spawn_next(data)
            return

        self._set_block_pose(self.active_color, data, self.active_position)

    def sync_poses(self, data: mujoco.MjData) -> None:
        for color in BLOCK_COLORS:
            if color != self.active_color:
                stored = self.stored_positions[color]
                self._set_block_pose(color, data, stored if stored is not None else self.park_positions[color])

        if self.active_color is None or self.active_position is None:
            return

        if self.attached:
            self._set_block_pose(self.active_color, data, self.active_position, self.active_quat)
            return

        self._set_block_pose(self.active_color, data, self.active_position)

    def active_block_position(self) -> np.ndarray | None:
        if self.active_position is None:
            return None
        return self.active_position.copy()

    def status_summary(self) -> str:
        motion = "running" if self.motion_enabled else "paused"
        if self.attached:
            motion = "attached"
        if self.active_color is None or self.active_position is None:
            return f"conveyor={motion} idle"
        return (
            f"conveyor={motion} active={self.active_color}#{self.active_spawn_id} "
            f"pos=({self.active_position[0]:+.3f}, {self.active_position[1]:+.3f}, {self.active_position[2]:+.3f}) "
            f"spawned={self.spawn_count} completed={self.completed_blocks}"
        )


class ArmSequencer:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self.tcp_site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "arm_tcp_site"))
        self.joint_ids = {
            name: int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{ARM_PREFIX}{name}")
            )
            for name in ARM_JOINT_NAMES
        }
        self.qpos_adrs = {
            name: int(model.jnt_qposadr[joint_id])
            for name, joint_id in self.joint_ids.items()
        }
        self.dof_adrs = {
            name: int(model.jnt_dofadr[joint_id])
            for name, joint_id in self.joint_ids.items()
        }
        self.path_queue: list[MotionSegment] = []
        self.active_segment: MotionSegment | None = None
        self.segment_elapsed = 0.0
        self.segment_start_q: np.ndarray | None = None
        self.hold_q = self.current_q()

    def current_q(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_adrs[name]] for name in ARM_JOINT_NAMES], dtype=np.float64)

    def tcp_position(self) -> np.ndarray:
        return np.array(self.data.site_xpos[self.tcp_site_id], dtype=np.float64)

    def tcp_rotation(self) -> np.ndarray:
        return np.array(self.data.site_xmat[self.tcp_site_id], dtype=np.float64).reshape(3, 3)

    def set_pose(self, joint_values: tuple[float, ...] | list[float] | np.ndarray) -> None:
        self.hold_q = np.array(joint_values, dtype=np.float64)
        for joint_name, joint_value in zip(ARM_JOINT_NAMES, joint_values):
            self.data.qpos[self.qpos_adrs[joint_name]] = float(joint_value)
            self.data.qvel[self.dof_adrs[joint_name]] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def apply_named_pose(self, pose_name: str) -> None:
        self.set_pose(ARM_POSES[pose_name])

    def start_named_path(self, segments: list[tuple[str, float]]) -> None:
        self.path_queue = [
            MotionSegment(
                name=pose_name,
                target_q=np.array(ARM_POSES[pose_name], dtype=np.float64),
                duration=max(0.02, float(duration)),
            )
            for pose_name, duration in segments
        ]
        self.active_segment = None
        self.segment_elapsed = 0.0
        self.segment_start_q = None
        self._start_next_segment()

    def _start_next_segment(self) -> None:
        if not self.path_queue:
            self.active_segment = None
            self.segment_start_q = None
            self.segment_elapsed = 0.0
            return
        self.active_segment = self.path_queue.pop(0)
        self.segment_start_q = self.current_q()
        self.segment_elapsed = 0.0

    def update(self, dt: float) -> None:
        if self.active_segment is None or self.segment_start_q is None:
            self.set_pose(self.hold_q)
            return
        self.segment_elapsed += dt
        phase = min(1.0, self.segment_elapsed / self.active_segment.duration)
        target_q = lerp_pose(
            self.segment_start_q,
            self.active_segment.target_q,
            smoothstep(phase),
        )
        self.set_pose(target_q)
        if phase >= 1.0:
            self.set_pose(self.active_segment.target_q)
            self._start_next_segment()

    def is_busy(self) -> bool:
        return self.active_segment is not None

    def status_summary(self) -> str:
        if self.active_segment is None:
            return "arm=idle"
        phase = min(1.0, self.segment_elapsed / self.active_segment.duration)
        return f"arm={self.active_segment.name} {phase:.0%}"


class VisionDetector:
    """Capture the MuJoCo camera image and detect conveyor blocks with OpenCV."""

    def __init__(self, model: mujoco.MjModel, config: DemoConfig):
        self.model = model
        self.config = config
        self.renderer: mujoco.Renderer | None = None
        self.available = False
        self.failure_message: str | None = None
        self.frames_captured = 0
        self.last_shape: tuple[int, ...] | None = None
        self.background_rgb: np.ndarray | None = None
        self.last_frame_rgb: np.ndarray | None = None
        self.last_debug_bgr: np.ndarray | None = None
        self.last_detections: list[ColorDetection] = []
        self.last_primary_detection: ColorDetection | None = None
        self.last_roi_pixels: tuple[int, int, int, int] | None = None

    def _render(self, data: mujoco.MjData):
        if not self.config.capture_camera:
            return None
        if self.renderer is None and self.failure_message is None:
            try:
                self.renderer = mujoco.Renderer(
                    self.model,
                    height=self.config.camera_height,
                    width=self.config.camera_width,
                )
                self.available = True
            except Exception as exc:  # pragma: no cover - depends on GL runtime
                self.failure_message = str(exc)
                self.available = False
                return None
        if not self.available or self.renderer is None:
            return None

        self.renderer.update_scene(data, camera=self.config.camera_name)
        frame = self.renderer.render()
        self.frames_captured += 1
        self.last_shape = tuple(frame.shape)
        return frame

    def capture_background(self, data: mujoco.MjData):
        frame = self._render(data)
        if frame is None:
            return None
        self.background_rgb = frame.copy()
        self.last_frame_rgb = frame
        self.last_debug_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.last_detections = []
        self.last_primary_detection = None
        return frame

    def _roi_bounds(self, frame: np.ndarray) -> tuple[int, int, int, int]:
        height, width = frame.shape[:2]
        x_min_ratio, y_min_ratio, x_max_ratio, y_max_ratio = self.config.vision_roi
        x0 = max(0, min(width - 1, int(width * x_min_ratio)))
        y0 = max(0, min(height - 1, int(height * y_min_ratio)))
        x1 = max(x0 + 1, min(width, int(width * x_max_ratio)))
        y1 = max(y0 + 1, min(height, int(height * y_max_ratio)))
        return x0, y0, x1, y1

    def _foreground_mask(self, frame: np.ndarray, roi: tuple[int, int, int, int]) -> np.ndarray:
        x0, y0, x1, y1 = roi
        if self.background_rgb is None:
            return np.full((y1 - y0, x1 - x0), 255, dtype=np.uint8)

        diff = cv2.absdiff(frame[y0:y1, x0:x1], self.background_rgb[y0:y1, x0:x1])
        diff_gray = cv2.cvtColor(diff, cv2.COLOR_RGB2GRAY)
        _, foreground_mask = cv2.threshold(
            diff_gray,
            self.config.foreground_threshold,
            255,
            cv2.THRESH_BINARY,
        )
        kernel = np.ones((5, 5), dtype=np.uint8)
        foreground_mask = cv2.morphologyEx(foreground_mask, cv2.MORPH_OPEN, kernel)
        foreground_mask = cv2.morphologyEx(foreground_mask, cv2.MORPH_CLOSE, kernel)
        return foreground_mask

    def _color_mask(self, hsv_frame: np.ndarray, label: str) -> np.ndarray:
        mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for lower, upper in HSV_COLOR_RANGES[label]:
            lower_np = np.array(lower, dtype=np.uint8)
            upper_np = np.array(upper, dtype=np.uint8)
            mask |= cv2.inRange(hsv_frame, lower_np, upper_np)
        return mask

    def detect(self, frame: np.ndarray) -> list[ColorDetection]:
        roi = self._roi_bounds(frame)
        x0, y0, x1, y1 = roi
        self.last_roi_pixels = roi
        roi_frame = frame[y0:y1, x0:x1]
        hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2HSV)
        foreground_mask = self._foreground_mask(frame, roi)
        detections: list[ColorDetection] = []
        kernel = np.ones((5, 5), dtype=np.uint8)

        for label in BLOCK_COLORS:
            color_mask = self._color_mask(hsv_frame, label)
            combined_mask = cv2.bitwise_and(color_mask, foreground_mask)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                combined_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.config.color_min_area or area > self.config.color_max_area:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                moments = cv2.moments(contour)
                if moments["m00"] > 0:
                    center_x = int(moments["m10"] / moments["m00"])
                    center_y = int(moments["m01"] / moments["m00"])
                else:
                    center_x = x + width // 2
                    center_y = y + height // 2
                detections.append(
                    ColorDetection(
                        label=label,
                        bbox=(x + x0, y + y0, width, height),
                        center=(center_x + x0, center_y + y0),
                        area=area,
                    )
                )

        detections.sort(key=lambda detection: detection.area, reverse=True)
        self.last_detections = detections
        self.last_primary_detection = detections[0] if detections else None

        debug_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.rectangle(debug_frame, (x0, y0), (x1, y1), (220, 220, 220), 1)
        for detection in detections:
            x, y, width, height = detection.bbox
            color = DETECTION_BGR[detection.label]
            cv2.rectangle(debug_frame, (x, y), (x + width, y + height), color, 2)
            cv2.circle(debug_frame, detection.center, 4, color, -1)
            cv2.putText(
                debug_frame,
                detection.summary(),
                (x, max(18, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        self.last_debug_bgr = debug_frame
        return detections

    def capture(self, data: mujoco.MjData):
        frame = self._render(data)
        if frame is None:
            return None
        self.last_frame_rgb = frame
        self.detect(frame)
        return frame

    def detections_summary(self) -> str:
        if self.failure_message:
            return f"vision unavailable: {self.failure_message}"
        if not self.last_detections:
            return "no block detected"
        return ", ".join(detection.summary() for detection in self.last_detections[:3])


class DemoController:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: DemoConfig):
        self.model = model
        self.data = data
        self.config = config
        self.arm = ArmSequencer(model, data)
        self.conveyor = ConveyorManager(model, config)
        self.vision = VisionDetector(model, config)
        self.step_count = 0
        self.initialized = False
        self.state = DemoState.SPAWN
        self.state_elapsed = 0.0
        self.current_sort_color: str | None = None
        self.handled_spawn_id: int | None = None

    def initialize(self) -> None:
        self.arm.apply_named_pose("ready_open")
        self.conveyor.initialize(self.data)
        mujoco.mj_forward(self.model, self.data)
        self.vision.capture_background(self.data)
        self._transition(DemoState.SPAWN)
        self.initialized = True

    def _dur(self, base_duration: float, minimum: float = 0.06) -> float:
        return max(minimum, base_duration * self.config.motion_duration_scale)

    def _transition(self, new_state: DemoState) -> None:
        self.state = new_state
        self.state_elapsed = 0.0

        if new_state == DemoState.SPAWN:
            self.conveyor.resume()
            self.current_sort_color = None
        elif new_state == DemoState.CONVEY:
            self.conveyor.resume()
        elif new_state == DemoState.TRACK:
            self.conveyor.resume()
        elif new_state == DemoState.STOP_FOR_PICK:
            self.conveyor.pause()
            self.current_sort_color = self.conveyor.active_color
            self.handled_spawn_id = self.conveyor.active_spawn_id
        elif new_state == DemoState.MOVE_TO_PICK:
            self.arm.start_named_path(
                [
                    ("pre_pick_open", self._dur(0.22)),
                    ("pick_open", self._dur(0.18)),
                ]
            )
        elif new_state == DemoState.GRASP:
            self.arm.start_named_path([("pick_closed", self._dur(0.16, minimum=0.05))])
        elif new_state == DemoState.LIFT:
            self.arm.start_named_path([("lift_closed", self._dur(0.24))])
        elif new_state == DemoState.MOVE_TO_PLACE:
            color = self.current_sort_color or "red"
            self.arm.start_named_path(
                [
                    (f"pre_place_{color}_closed", self._dur(0.34)),
                    (f"place_{color}_closed", self._dur(0.20)),
                ]
            )
        elif new_state == DemoState.RELEASE:
            color = self.current_sort_color or "red"
            self.arm.start_named_path([(f"place_{color}_open", self._dur(0.16, minimum=0.05))])
        elif new_state == DemoState.RETURN_HOME:
            color = self.current_sort_color or "red"
            self.arm.start_named_path(
                [
                    (f"pre_place_{color}_open", self._dur(0.18)),
                    ("lift_open", self._dur(0.24)),
                    ("ready_open", self._dur(0.26)),
                ]
            )
        elif new_state == DemoState.RESUME:
            self.conveyor.pause()

    def _has_active_color_detection(self) -> bool:
        active_color = self.conveyor.active_color
        if active_color is None:
            return False
        return any(detection.label == active_color for detection in self.vision.last_detections)

    def _should_pause_for_pick(self) -> bool:
        active_color = self.conveyor.active_color
        active_spawn_id = self.conveyor.active_spawn_id
        active_position = self.conveyor.active_block_position()
        if active_color is None or active_spawn_id is None or active_position is None:
            return False
        if active_spawn_id == self.handled_spawn_id:
            return False
        if not self._has_active_color_detection():
            return False
        pick_x = float(self.conveyor.pick_position[0])
        return abs(float(active_position[0]) - pick_x) <= self.config.pick_x_tolerance

    def _update_tracking_states(self) -> None:
        if self.conveyor.active_color is None:
            if self.state != DemoState.SPAWN:
                self._transition(DemoState.SPAWN)
            return

        if self._should_pause_for_pick():
            self._transition(DemoState.STOP_FOR_PICK)
            return

        target_state = DemoState.TRACK if self._has_active_color_detection() else DemoState.CONVEY
        if self.state != target_state:
            self._transition(target_state)

    def _update_state_machine(self) -> None:
        if self.state in {DemoState.SPAWN, DemoState.CONVEY, DemoState.TRACK}:
            self._update_tracking_states()
            return

        if self.state == DemoState.STOP_FOR_PICK:
            if self.state_elapsed >= self.config.stop_pause_duration:
                self._transition(DemoState.MOVE_TO_PICK)
            return

        if self.state == DemoState.MOVE_TO_PICK and not self.arm.is_busy():
            self._transition(DemoState.GRASP)
            return

        if self.state == DemoState.GRASP and not self.arm.is_busy():
            self.conveyor.attach_active_block(
                self.data,
                self.arm.tcp_position(),
                self.arm.tcp_rotation(),
            )
            self._transition(DemoState.LIFT)
            return

        if self.state == DemoState.LIFT and not self.arm.is_busy():
            self._transition(DemoState.MOVE_TO_PLACE)
            return

        if self.state == DemoState.MOVE_TO_PLACE and not self.arm.is_busy():
            self._transition(DemoState.RELEASE)
            return

        if self.state == DemoState.RELEASE and not self.arm.is_busy():
            self.conveyor.store_active_block(self.data)
            self._transition(DemoState.RETURN_HOME)
            return

        if self.state == DemoState.RETURN_HOME and not self.arm.is_busy():
            self._transition(DemoState.RESUME)
            return

        if self.state == DemoState.RESUME and self.state_elapsed >= self.config.resume_pause_duration:
            self.current_sort_color = None
            self._transition(DemoState.CONVEY)

    def step(self) -> None:
        if not self.initialized:
            self.initialize()

        self.arm.update(self.config.control_dt)
        if self.conveyor.attached:
            self.conveyor.update_attached_block(
                self.data,
                self.arm.tcp_position(),
                self.arm.tcp_rotation(),
            )
        previous_spawn_id = self.conveyor.active_spawn_id
        self.conveyor.update(self.model, self.data)
        if self.conveyor.active_spawn_id != previous_spawn_id:
            self.vision.last_detections = []
            self.vision.last_primary_detection = None
        mujoco.mj_step(self.model, self.data)
        self.arm.set_pose(self.arm.hold_q)
        self.conveyor.sync_poses(self.data)
        mujoco.mj_forward(self.model, self.data)
        self.step_count += 1

        if self.step_count % self.config.camera_stride == 0:
            self.vision.capture(self.data)

        self.state_elapsed += self.config.control_dt
        self._update_state_machine()

    def run_headless(self, steps: int) -> None:
        for _ in range(steps):
            self.step()

    def summary(self) -> str:
        camera_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_id)
            for camera_id in range(self.model.ncam)
        ]
        return (
            f"model={self.model.nbody} bodies, {self.model.njnt} joints, "
            f"{self.model.ncam} cameras {camera_names}"
        )

    def status_summary(self) -> str:
        sort_color = self.current_sort_color or "-"
        return (
            f"state={self.state.name} sort={sort_color} | "
            f"{self.arm.status_summary()} | "
            f"{self.conveyor.status_summary()} | "
            f"vision={self.vision.detections_summary()}"
        )
