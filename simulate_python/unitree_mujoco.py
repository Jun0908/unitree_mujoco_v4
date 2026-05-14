import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()
last_step_debug_time = 0.0

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)


if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model, last_step_debug_time

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        unitree.ApplyLocalLowCmdIfAvailable()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        if step_start - last_step_debug_time >= 1.0 and mj_model.nu > 16:
            print(
                "step_debug "
                f"time={mj_data.time:+.3f} "
                f"qpos16={mj_data.qpos[23]:+.3f} "
                f"qvel16={mj_data.qvel[22]:+.3f} "
                f"ctrl16={mj_data.ctrl[16]:+.3f} "
                f"sensor16={mj_data.sensordata[16]:+.3f}"
            )
            last_step_debug_time = step_start

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
