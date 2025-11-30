"""
LQR-controlled point-to-point flight using CtrlAviary and DSLQRControl.

The drone starts near (0, 0, ~0.1) and moves to the desired position
using a full-state LQR controller defined in DSLQRControl.py.
"""

import argparse
import time

import numpy as np
from gym_pybullet_drones.control.DSLQRControl import DSLQRControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import str2bool, sync

# ----------------------------------------------------------------------
# DEFAULTS
# ----------------------------------------------------------------------
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 7
DEFAULT_OUTPUT_FOLDER = "results"
DEFAULT_COLAB = False


def run(
    drone=DEFAULT_DRONES,
    num_drones=DEFAULT_NUM_DRONES,
    physics=DEFAULT_PHYSICS,
    gui=DEFAULT_GUI,
    record_video=DEFAULT_RECORD_VISION,
    plot=DEFAULT_PLOT,
    user_debug_gui=DEFAULT_USER_DEBUG_GUI,
    obstacles=DEFAULT_OBSTACLES,
    simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
    control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
    duration_sec=DEFAULT_DURATION_SEC,
    output_folder=DEFAULT_OUTPUT_FOLDER,
    colab=DEFAULT_COLAB,
):
    # ------------------------------------------------------------------
    # Initial conditions (same style as your PID script)
    # ------------------------------------------------------------------
    H = 0.1
    H_STEP = 0.05
    R = 0.3
    INIT_XYZS = np.array(
        [
            [
                R * np.cos((i / 6) * 2 * np.pi + np.pi / 2),
                R * np.sin((i / 6) * 2 * np.pi + np.pi / 2) - R,
                H + i * H_STEP,
            ]
            for i in range(num_drones)
        ]
    )
    INIT_RPYS = np.array(
        [[0.0, 0.0, i * (np.pi / 2) / num_drones] for i in range(num_drones)]
    )

    # Target position (same as your PID demo)
    destination_state = {
        "pos": np.array([1.0, 1.11, 1.23]),
        "vel": np.array([0.0, 0.0, 0.0]),
        "acc": np.array([0.0, 0.0, 0.0]),
        "jerk": np.array([0.0, 0.0, 0.0]),
        "yaw": 0.0,
        "yawdot": 0.0,
    }

    # ------------------------------------------------------------------
    # Environment
    # ------------------------------------------------------------------
    env = CtrlAviary(
        drone_model=drone,
        num_drones=num_drones,
        initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS,
        physics=physics,
        neighbourhood_radius=10,
        pyb_freq=simulation_freq_hz,
        ctrl_freq=control_freq_hz,
        gui=gui,
        record=record_video,
        obstacles=obstacles,
        user_debug_gui=user_debug_gui,
    )

    logger = Logger(
        logging_freq_hz=control_freq_hz,
        num_drones=num_drones,
        output_folder=output_folder,
        colab=colab,
    )

    # ------------------------------------------------------------------
    # Controllers
    # ------------------------------------------------------------------
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLQRControl(drone_model=drone) for _ in range(num_drones)]
    else:
        print("[ERROR] LQR example only supports CF2X / CF2P")
        env.close()
        return

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------
    action = np.zeros((num_drones, 4))
    START = time.time()

    for i in range(0, int(duration_sec * env.CTRL_FREQ)):
        obs, reward, terminated, truncated, info = env.step(action)

        pos = destination_state["pos"]
        vel = destination_state["vel"]
        acc = destination_state["acc"]

        for j in range(num_drones):
            # Desired yaw kept at initial yaw (or you can set to destination_state["yaw"])
            des_rpy = np.array([0.0, 0.0, INIT_RPYS[j, 2]])

            action[j, :], _, des_rpy = ctrl[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs[j],
                target_pos=pos,
                target_vel=vel,
                target_rpy=des_rpy,
                target_acc=acc,
            )

        # Log
        for j in range(num_drones):
            logger.log(
                drone=j,
                timestamp=i / env.CTRL_FREQ,
                state=obs[j],
                des_state=np.hstack([pos, vel, des_rpy, np.array([0.0, 0.0, 0.0])]),
                control=np.hstack([pos, des_rpy, np.zeros(6)]),
            )

        # Render and real-time sync
        env.render()
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    env.close()

    if plot:
        logger.plot()


if __name__ == "__main__":
    import math  # needed for np.pi above

    parser = argparse.ArgumentParser(
        description="LQR position control using CtrlAviary and DSLQRControl"
    )
    parser.add_argument("--drone", default=DEFAULT_DRONES, type=DroneModel)
    parser.add_argument("--num_drones", default=DEFAULT_NUM_DRONES, type=int)
    parser.add_argument("--physics", default=DEFAULT_PHYSICS, type=Physics)
    parser.add_argument("--gui", default=DEFAULT_GUI, type=str2bool)
    parser.add_argument("--record_video", default=DEFAULT_RECORD_VISION, type=str2bool)
    parser.add_argument("--plot", default=DEFAULT_PLOT, type=str2bool)
    parser.add_argument(
        "--user_debug_gui", default=DEFAULT_USER_DEBUG_GUI, type=str2bool
    )
    parser.add_argument("--obstacles", default=DEFAULT_OBSTACLES, type=str2bool)
    parser.add_argument(
        "--simulation_freq_hz", default=DEFAULT_SIMULATION_FREQ_HZ, type=int
    )
    parser.add_argument("--control_freq_hz", default=DEFAULT_CONTROL_FREQ_HZ, type=int)
    parser.add_argument("--duration_sec", default=DEFAULT_DURATION_SEC, type=int)
    parser.add_argument("--output_folder", default=DEFAULT_OUTPUT_FOLDER, type=str)
    parser.add_argument("--colab", default=DEFAULT_COLAB, type=bool)

    ARGS = parser.parse_args()
    run(**vars(ARGS))
