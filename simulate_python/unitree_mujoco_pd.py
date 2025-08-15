import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand
from armteach import action_replay

import config

KPs = [60] * 7
KDs = [1] * 7
locker = threading.Lock()

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
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()
    
    last_action_time = time.perf_counter()  
    
    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )

        mj_data.qpos[0:3] = [0,0,0.8]
        mj_data.qpos[3:7] = [1,0,0,0]
        mj_data.qpos[7:22] = [0] * 15

        # 计算当前时间与上次执行时间的差
        current_time = time.perf_counter()
        if current_time - last_action_time >= 0.02: 
            try:
                frame = next(frames)
            except StopIteration:
                print("已到文件末尾，回放结束")
            arm_action_callback(frame)  
            last_action_time = current_time  

        # 执行物理仿真
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        # 控制仿真步骤的时间
        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)



def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)

def arm_action_callback(frame):
    """Callback function to handle arm actions during replay."""
    
    for i, angle in enumerate(frame[0]):
        target_dof_pos[i] = angle
        tau = pd_control(target_dof_pos, mj_data.qpos[22:29], KPs, np.zeros_like(KDs), mj_data.qvel[21:28], KDs)
        mj_data.ctrl[15:22] = tau

    for i, angle in enumerate(frame[1]):
        target_dof_pos[i] = angle
        tau = pd_control(target_dof_pos, mj_data.qpos[29:36], KPs, np.zeros_like(KDs), mj_data.qvel[28:35], KDs)
        mj_data.ctrl[22:29] = tau

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


if __name__ == "__main__":
    frames = action_replay.iter_arm_frames("/home/ris/unitree_mujoco/test.act")
    target_dof_pos = [0] * 7
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
