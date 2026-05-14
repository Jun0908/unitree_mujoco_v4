from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from sorting_demo import (
    DEFAULT_ARM_MODEL,
    DEFAULT_BASE_SCENE,
    DEFAULT_SCENE_EXPORT,
    DemoConfig,
    DemoController,
    build_demo_model,
    export_compiled_scene,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="MuJoCo conveyor sorting demo scaffold (Plan Step 1-7)."
    )
    parser.add_argument("--base-scene", type=Path, default=DEFAULT_BASE_SCENE)
    parser.add_argument("--arm-model", type=Path, default=DEFAULT_ARM_MODEL)
    parser.add_argument(
        "--export-scene-path",
        type=Path,
        default=None,
        help="Write the combined MJCF scene with the attached SO-101 arm.",
    )
    parser.add_argument(
        "--headless-steps",
        type=int,
        default=0,
        help="Run a fixed number of simulation steps without the GUI viewer.",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Compile and initialize the scene without opening the passive viewer.",
    )
    parser.add_argument("--camera", default="sorting_cam")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument(
        "--disable-camera",
        action="store_true",
        help="Skip offscreen camera capture in the controller loop.",
    )
    return parser.parse_args()


def configure_viewer_camera(viewer) -> None:
    viewer.cam.lookat[:] = [0.08, -0.09, 0.24]
    viewer.cam.distance = 1.08
    viewer.cam.azimuth = 136
    viewer.cam.elevation = -23


def run_viewer(controller: DemoController) -> None:
    with mujoco.viewer.launch_passive(
        controller.model,
        controller.data,
        show_left_ui=True,
        show_right_ui=False,
    ) as viewer:
        configure_viewer_camera(viewer)
        next_tick = time.perf_counter()
        while viewer.is_running():
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            controller.step()
            viewer.sync()
            next_tick += controller.config.control_dt


def main() -> None:
    args = parse_args()

    if args.export_scene_path is not None:
        output_path = export_compiled_scene(
            output_path=args.export_scene_path,
            base_scene_path=args.base_scene,
            arm_model_path=args.arm_model,
        )
        print(f"Exported combined scene to {output_path}")

    model, data = build_demo_model(
        base_scene_path=args.base_scene,
        arm_model_path=args.arm_model,
    )
    controller = DemoController(
        model,
        data,
        DemoConfig(
            camera_name=args.camera,
            camera_width=args.camera_width,
            camera_height=args.camera_height,
            capture_camera=not args.disable_camera,
        ),
    )

    print("Sorting demo scaffold ready.")
    print(f"Base scene: {args.base_scene}")
    print(f"Arm model:  {args.arm_model}")
    print(controller.summary())

    if args.export_scene_path is None:
        print(f"Use --export-scene-path {DEFAULT_SCENE_EXPORT} to inspect the combined MJCF.")

    if args.no_viewer or args.headless_steps > 0:
        steps = max(1, args.headless_steps)
        controller.run_headless(steps)
        print(
            "Headless run complete: "
            f"{steps} steps, sim_time={controller.data.time:.3f}s, "
            f"captured_frames={controller.vision.frames_captured}"
        )
        if controller.vision.failure_message:
            print(f"Camera capture skipped: {controller.vision.failure_message}")
        elif controller.vision.last_shape is not None:
            print(f"Last camera frame shape: {controller.vision.last_shape}")
        print(controller.status_summary())
        return

    run_viewer(controller)


if __name__ == "__main__":
    main()
