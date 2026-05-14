from __future__ import annotations


GRIPPER_OPEN = 1.10
GRIPPER_CLOSED = 0.12


def with_gripper(pose: tuple[float, float, float, float, float], gripper: float) -> tuple[float, ...]:
    return (*pose, gripper)


HOME_CORE = (-0.020040, -1.347550, 1.169730, 0.112718, 1.765920)
READY_CORE = HOME_CORE
PRE_PICK_CORE = (-0.021084, -1.276616, 1.202642, 0.169630, 1.668805)
PICK_CORE = (-0.034758, -1.099324, 1.215053, 0.158321, 1.703000)
LIFT_CORE = (0.015406, -1.624970, 1.262436, 0.332317, 1.631694)

PRE_PLACE_RED_CORE = (-1.118846, -1.297842, 1.085744, 0.454647, 1.703000)
PLACE_RED_CORE = (-1.118846, -1.172563, 1.212809, 0.426251, 1.703000)
PRE_PLACE_BLUE_CORE = (-0.072311, -1.623048, 0.560043, 1.647218, 1.703000)
PLACE_BLUE_CORE = (-0.072289, -1.601691, 0.833877, 1.658060, 1.703000)
PRE_PLACE_YELLOW_CORE = (1.044214, -1.073818, 0.772067, 0.762003, 1.703000)
PLACE_YELLOW_CORE = (1.044214, -0.922278, 0.891050, 0.726425, 1.703000)


ARM_POSES = {
    "home_open": with_gripper(HOME_CORE, GRIPPER_OPEN),
    "ready_open": with_gripper(READY_CORE, GRIPPER_OPEN),
    "pre_pick_open": with_gripper(PRE_PICK_CORE, GRIPPER_OPEN),
    "pick_open": with_gripper(PICK_CORE, GRIPPER_OPEN),
    "pick_closed": with_gripper(PICK_CORE, GRIPPER_CLOSED),
    "lift_open": with_gripper(LIFT_CORE, GRIPPER_OPEN),
    "lift_closed": with_gripper(LIFT_CORE, GRIPPER_CLOSED),
    "pre_place_red_open": with_gripper(PRE_PLACE_RED_CORE, GRIPPER_OPEN),
    "pre_place_red_closed": with_gripper(PRE_PLACE_RED_CORE, GRIPPER_CLOSED),
    "place_red_open": with_gripper(PLACE_RED_CORE, GRIPPER_OPEN),
    "place_red_closed": with_gripper(PLACE_RED_CORE, GRIPPER_CLOSED),
    "pre_place_blue_open": with_gripper(PRE_PLACE_BLUE_CORE, GRIPPER_OPEN),
    "pre_place_blue_closed": with_gripper(PRE_PLACE_BLUE_CORE, GRIPPER_CLOSED),
    "place_blue_open": with_gripper(PLACE_BLUE_CORE, GRIPPER_OPEN),
    "place_blue_closed": with_gripper(PLACE_BLUE_CORE, GRIPPER_CLOSED),
    "pre_place_yellow_open": with_gripper(PRE_PLACE_YELLOW_CORE, GRIPPER_OPEN),
    "pre_place_yellow_closed": with_gripper(PRE_PLACE_YELLOW_CORE, GRIPPER_CLOSED),
    "place_yellow_open": with_gripper(PLACE_YELLOW_CORE, GRIPPER_OPEN),
    "place_yellow_closed": with_gripper(PLACE_YELLOW_CORE, GRIPPER_CLOSED),
}
