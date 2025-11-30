"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python trajectory_tracking.py

Notes
-----
The drones move along cicular or diamond trajectories
in the space.

"""

# import argparse
# import math
# import os
# import pdb
# import random
# import time
# from datetime import datetime

# import matplotlib.pyplot as plt
# import numpy as np
# import pybullet as p
# from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
# from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
# from gym_pybullet_drones.trajectory import circle, diamond
# from gym_pybullet_drones.utils.enums import DroneModel, Physics
# from gym_pybullet_drones.utils.Logger import Logger
# from gym_pybullet_drones.utils.utils import str2bool, sync

# DEFAULT_DRONES = DroneModel("cf2x")
# DEFAULT_NUM_DRONES = 1
# DEFAULT_PHYSICS = Physics("pyb")
# DEFAULT_GUI = True
# DEFAULT_RECORD_VISION = False
# DEFAULT_PLOT = True
# DEFAULT_USER_DEBUG_GUI = False
# DEFAULT_OBSTACLES = False
# DEFAULT_SIMULATION_FREQ_HZ = 240
# DEFAULT_CONTROL_FREQ_HZ = 48
# DEFAULT_DURATION_SEC = 15
# DEFAULT_OUTPUT_FOLDER = "results"
# DEFAULT_COLAB = False


# def run(
#     drone=DEFAULT_DRONES,
#     num_drones=DEFAULT_NUM_DRONES,
#     physics=DEFAULT_PHYSICS,
#     gui=DEFAULT_GUI,
#     record_video=DEFAULT_RECORD_VISION,
#     plot=DEFAULT_PLOT,
#     user_debug_gui=DEFAULT_USER_DEBUG_GUI,
#     obstacles=DEFAULT_OBSTACLES,
#     simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
#     control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
#     duration_sec=DEFAULT_DURATION_SEC,
#     output_folder=DEFAULT_OUTPUT_FOLDER,
#     colab=DEFAULT_COLAB,
# ):
#     #### Initialize the simulation #############################
#     H = 0.1
#     H_STEP = 0.05
#     R = 0.3
#     INIT_XYZS = np.array(
#         [
#             [
#                 R * np.cos((i / 6) * 2 * np.pi + np.pi / 2),
#                 R * np.sin((i / 6) * 2 * np.pi + np.pi / 2) - R,
#                 H + i * H_STEP,
#             ]
#             for i in range(num_drones)
#         ]
#     )
#     INIT_RPYS = np.array(
#         [[0, 0, i * (np.pi / 2) / num_drones] for i in range(num_drones)]
#     )

#     #### Initialize a circular trajectory ######################
#     PERIOD = 8
#     NUM_WP = control_freq_hz * PERIOD
#     TARGET_POS = np.zeros((NUM_WP, 3))
#     for i in range(NUM_WP):
#         TARGET_POS[i, :] = (
#             R * np.cos((i / NUM_WP) * (2 * np.pi) + np.pi / 2) + INIT_XYZS[0, 0],
#             R * np.sin((i / NUM_WP) * (2 * np.pi) + np.pi / 2) - R + INIT_XYZS[0, 1],
#             H,
#         )
#     wp_counters = np.array([0])

#     #### Create the environment ################################
#     env = CtrlAviary(
#         drone_model=drone,
#         num_drones=num_drones,
#         initial_xyzs=INIT_XYZS,
#         initial_rpys=INIT_RPYS,
#         physics=physics,
#         neighbourhood_radius=10,
#         pyb_freq=simulation_freq_hz,
#         ctrl_freq=control_freq_hz,
#         gui=gui,
#         record=record_video,
#         obstacles=obstacles,
#         user_debug_gui=user_debug_gui,
#     )

#     #### Obtain the PyBullet Client ID from the environment ####
#     PYB_CLIENT = env.getPyBulletClient()

#     #### Initialize the logger #################################
#     logger = Logger(
#         logging_freq_hz=control_freq_hz,
#         num_drones=num_drones,
#         output_folder=output_folder,
#         colab=colab,
#     )

#     #### Initialize the controllers ############################
#     if drone in [DroneModel.CF2X, DroneModel.CF2P]:
#         ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

#     #### Run the simulation ####################################
#     action = np.zeros((num_drones, 4))
#     START = time.time()
#     elapsed = 0
#     for i in range(0, int(duration_sec * env.CTRL_FREQ)):
#         #### Step the simulation ###################################
#         obs, reward, terminated, truncated, info = env.step(action)

#         # des_st = circle.circle(elapsed)
#         des_st = diamond.diamond(elapsed)
#         pos = des_st["pos"]
#         vel = des_st["vel"]
#         acc = des_st["acc"]
#         #### Compute control for the current way point #############
#         for j in range(num_drones):
#             action[j, :], _, des_rpy = ctrl[j].computeControlFromState(
#                 control_timestep=env.CTRL_TIMESTEP,
#                 state=obs[j],
#                 target_pos=pos,
#                 target_vel=vel,
#                 target_rpy=INIT_RPYS[j, :],
#                 target_acc=acc,
#             )
#         #### Go to the next way point and loop #####################
#         for j in range(num_drones):
#             wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP - 1) else 0

#         #### Log the simulation ####################################
#         for j in range(num_drones):
#             logger.log(
#                 drone=j,
#                 timestamp=i / env.CTRL_FREQ,
#                 state=obs[j],
#                 des_state=np.hstack([pos, vel, des_rpy, np.array([0, 0, 0])]),
#                 control=np.hstack([pos, INIT_RPYS[j, :], np.zeros(6)]),
#             )

#         #### Printout ##############################################
#         env.render()

#         #### Sync the simulation ###################################
#         if gui:
#             sync(i, START, env.CTRL_TIMESTEP)

#         elapsed = elapsed + 1 / env.CTRL_FREQ

#     #### Close the environment #################################
#     env.close()

#     #### Save the simulation results ###########################
#     # logger.save()
#     # logger.save_as_csv("pid") # Optional CSV save

#     #### Plot the simulation results ###########################
#     if plot:
#         logger.plot()


# if __name__ == "__main__":
#     #### Define and parse (optional) arguments for the script ##
#     parser = argparse.ArgumentParser(
#         description="Helix flight script using CtrlAviary and DSLPIDControl"
#     )
#     parser.add_argument(
#         "--drone",
#         default=DEFAULT_DRONES,
#         type=DroneModel,
#         help="Drone model (default: CF2X)",
#         metavar="",
#         choices=DroneModel,
#     )
#     parser.add_argument(
#         "--num_drones",
#         default=DEFAULT_NUM_DRONES,
#         type=int,
#         help="Number of drones (default: 3)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--physics",
#         default=DEFAULT_PHYSICS,
#         type=Physics,
#         help="Physics updates (default: PYB)",
#         metavar="",
#         choices=Physics,
#     )
#     parser.add_argument(
#         "--gui",
#         default=DEFAULT_GUI,
#         type=str2bool,
#         help="Whether to use PyBullet GUI (default: True)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--record_video",
#         default=DEFAULT_RECORD_VISION,
#         type=str2bool,
#         help="Whether to record a video (default: False)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--plot",
#         default=DEFAULT_PLOT,
#         type=str2bool,
#         help="Whether to plot the simulation results (default: True)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--user_debug_gui",
#         default=DEFAULT_USER_DEBUG_GUI,
#         type=str2bool,
#         help="Whether to add debug lines and parameters to the GUI (default: False)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--obstacles",
#         default=DEFAULT_OBSTACLES,
#         type=str2bool,
#         help="Whether to add obstacles to the environment (default: True)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--simulation_freq_hz",
#         default=DEFAULT_SIMULATION_FREQ_HZ,
#         type=int,
#         help="Simulation frequency in Hz (default: 240)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--control_freq_hz",
#         default=DEFAULT_CONTROL_FREQ_HZ,
#         type=int,
#         help="Control frequency in Hz (default: 48)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--duration_sec",
#         default=DEFAULT_DURATION_SEC,
#         type=int,
#         help="Duration of the simulation in seconds (default: 5)",
#         metavar="",
#     )
#     parser.add_argument(
#         "--output_folder",
#         default=DEFAULT_OUTPUT_FOLDER,
#         type=str,
#         help='Folder where to save logs (default: "results")',
#         metavar="",
#     )
#     parser.add_argument(
#         "--colab",
#         default=DEFAULT_COLAB,
#         type=bool,
#         help='Whether example is being run by a notebook (default: "False")',
#         metavar="",
#     )
#     ARGS = parser.parse_args()

#     run(**vars(ARGS))


# ---------------------------------------------------------------------------------#
# ---------------------------------PHASE 3-----------------------------------------#
# ---------------------------------------------------------------------------------#

import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.trajectory import circle, diamond
from gym_pybullet_drones.trajectory.rmse_utils import RMSETracker
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import str2bool, sync


###############################################################################
# FIX FOR BLANK PNG SAVES
###############################################################################
def save_logger_plot(logger, save_path):
    plt.close("all")
    logger.plot()  # Logger creates a fresh figure
    fig = plt.gcf()  # Capture that figure
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


###############################################################################
# PID GAIN MODES
###############################################################################
def apply_gain_mode(ctrl, mode):
    if mode == "baseline":
        ctrl.Kp_pos = np.array([2, 2, 2])
        ctrl.Kd_pos = np.array([2.35, 2.35, 2.35])
        ctrl.Kp_att = np.array([4800, 4800, 4800])
        ctrl.Kd_att = np.array([600, 600, 600])

    elif mode == "low_kd":
        ctrl.Kp_pos = np.array([2, 2, 2])
        ctrl.Kd_pos = np.array([1.5, 1.5, 1.5])
        ctrl.Kp_att = np.array([4800, 4800, 4800])
        ctrl.Kd_att = np.array([350, 350, 350])

    elif mode == "low_kp":
        ctrl.Kp_pos = np.array([1.2, 1.2, 1.2])
        ctrl.Kd_pos = np.array([2.35, 2.35, 2.35])
        ctrl.Kp_att = np.array([3000, 3000, 3000])
        ctrl.Kd_att = np.array([600, 600, 600])

    else:
        raise ValueError("Unknown gain mode")


###############################################################################
# TRAJECTORY SELECTOR
###############################################################################
def get_traj(name):
    if name == "circle":
        return circle.circle
    elif name == "diamond":
        return diamond.diamond
    else:
        raise ValueError("Unknown trajectory name")


###############################################################################
# MAIN RUN FUNCTION
###############################################################################
def run(exp_mode, traj, gui, batch, tf):
    # ------------------ ENVIRONMENT SETUP ------------------ #
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0, 0, 0.5]]),
        initial_rpys=np.zeros((1, 3)),
        physics=Physics.PYB,
        neighbourhood_radius=10,
        pyb_freq=240,
        ctrl_freq=48,
        gui=str2bool(gui),
    )

    rmse = RMSETracker()
    logger = Logger(logging_freq_hz=48, num_drones=1)

    # Controller
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
    apply_gain_mode(ctrl, exp_mode)

    traj_fun = get_traj(traj)

    action = np.zeros((1, 4))
    elapsed = 0
    START = time.time()

    steps = int(float(tf) * env.CTRL_FREQ)

    for i in range(steps):
        obs, _, _, _, _ = env.step(action)

        # Desired state
        des = traj_fun(elapsed)
        pos, vel, acc = des["pos"], des["vel"], des["acc"]

        # RMSE update
        rmse.add(pos, obs[0][0:3])

        # Controller step
        action[0, :], _, des_rpy = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=obs[0],
            target_pos=pos,
            target_vel=vel,
            target_rpy=np.zeros(3),
            target_acc=acc,
        )

        # Logging
        logger.log(
            drone=0,
            timestamp=i / env.CTRL_FREQ,
            state=obs[0],
            des_state=np.hstack([pos, vel, des_rpy, np.zeros(3)]),
        )

        if str2bool(gui):
            sync(i, START, env.CTRL_TIMESTEP)

        elapsed += 1 / env.CTRL_FREQ

    env.close()

    # ------------------ PRINT RMSE (PARSED BY experiment_runner) ------------------ #
    results = rmse.compute()
    print("===================================================")
    print(f"RMSE_X: {results['rmse_x']} m")
    print(f"RMSE_Y: {results['rmse_y']} m")
    print(f"RMSE_Z: {results['rmse_z']} m")
    print(f"TOTAL RMSE: {results['rmse_total']} m")
    print("===================================================")

    # ------------------ SAVE PLOT ------------------ #
    if not str2bool(batch):
        logger.plot()
    else:
        fname = f"results/{exp_mode}_{traj}_{tf}s.png"
        save_logger_plot(logger, fname)
        print(f"[SAVED PLOT] {fname}")


###############################################################################
# CLI ARGUMENTS
###############################################################################
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--exp_mode", type=str, default="baseline")
    parser.add_argument("--traj", type=str, default="circle")
    parser.add_argument("--gui", type=str, default="true")
    parser.add_argument("--batch", type=str, default="false")
    parser.add_argument("--tf", type=float, default=15.0)

    args = parser.parse_args()
    run(args.exp_mode, args.traj, args.gui, args.batch, args.tf)
