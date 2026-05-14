from __future__ import annotations

from pathlib import Path

import mujoco


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASE_SCENE = REPO_ROOT / "assets" / "sorting_demo_base_scene.xml"
DEFAULT_ARM_MODEL = REPO_ROOT / "unitree_robots" / "so101" / "so101_follower.urdf"
DEFAULT_SCENE_EXPORT = REPO_ROOT / "assets" / "sorting_demo_scene.xml"

ARM_PREFIX = "arm_"
ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
ARM_HOME_QPOS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.35)


def build_demo_spec(
    base_scene_path: Path = DEFAULT_BASE_SCENE,
    arm_model_path: Path = DEFAULT_ARM_MODEL,
) -> mujoco.MjSpec:
    """Build the demo scene spec and attach the SO-101 arm loaded from URDF."""

    scene = mujoco.MjSpec.from_file(str(base_scene_path.resolve()))
    scene.modelname = "sorting_demo_scene"
    scene.compiler.meshdir = str(arm_model_path.resolve().parent)

    arm_spec = mujoco.MjSpec.from_file(str(arm_model_path.resolve()))
    arm_mount = scene.site("arm_mount")
    scene.attach(arm_spec, site=arm_mount, prefix=ARM_PREFIX)

    # Reserve a stable TCP marker now so the later pick/place steps can use it.
    scene.body("arm_gripper_frame_link").add_site(
        name="arm_tcp_site",
        pos=[0.0, 0.0, 0.0],
        size=[0.006],
        rgba=[0.0, 0.0, 0.0, 0.0],
        group=2,
    )
    return scene


def build_demo_model(
    base_scene_path: Path = DEFAULT_BASE_SCENE,
    arm_model_path: Path = DEFAULT_ARM_MODEL,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Compile the full demo scene into a model + data pair."""

    spec = build_demo_spec(base_scene_path=base_scene_path, arm_model_path=arm_model_path)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def export_compiled_scene(
    output_path: Path = DEFAULT_SCENE_EXPORT,
    base_scene_path: Path = DEFAULT_BASE_SCENE,
    arm_model_path: Path = DEFAULT_ARM_MODEL,
) -> Path:
    """Write the full combined MJCF scene to disk for inspection/debugging."""

    spec = build_demo_spec(base_scene_path=base_scene_path, arm_model_path=arm_model_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(spec.to_xml(), encoding="utf-8")
    return output_path
