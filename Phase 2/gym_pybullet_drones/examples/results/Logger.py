import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from cycler import cycler

os.environ["KMP_DUPLICATE_LIB_OK"] = "True"


class Logger(object):
    """Unified Logger that saves all drone state and control data into a single CSV."""

    ################################################################################
    def __init__(
        self,
        logging_freq_hz: int,
        output_folder: str = "results",
        num_drones: int = 1,
        duration_sec: int = 0,
        colab: bool = False,
    ):
        self.COLAB = colab
        self.OUTPUT_FOLDER = output_folder
        os.makedirs(self.OUTPUT_FOLDER, exist_ok=True)

        self.LOGGING_FREQ_HZ = logging_freq_hz
        self.NUM_DRONES = num_drones
        self.PREALLOCATED_ARRAYS = False if duration_sec == 0 else True
        self.counters = np.zeros(num_drones)
        self.timestamps = np.zeros((num_drones, duration_sec * self.LOGGING_FREQ_HZ))
        self.states = np.zeros((num_drones, 16, duration_sec * self.LOGGING_FREQ_HZ))
        self.des_states = np.zeros(
            (num_drones, 12, duration_sec * self.LOGGING_FREQ_HZ)
        )
        self.controls = np.zeros((num_drones, 12, duration_sec * self.LOGGING_FREQ_HZ))

    ################################################################################
    def log(self, drone: int, timestamp, state, des_state, control=np.zeros(12)):
        """Log one timestep of data for one drone."""
        if (
            drone < 0
            or drone >= self.NUM_DRONES
            or timestamp < 0
            or len(state) != 20
            or len(control) != 12
        ):
            print("[ERROR] in Logger.log(), invalid data")
            return

        current_counter = int(self.counters[drone])
        if current_counter >= self.timestamps.shape[1]:
            # Expand arrays if needed
            self.timestamps = np.concatenate(
                (self.timestamps, np.zeros((self.NUM_DRONES, 1))), axis=1
            )
            self.states = np.concatenate(
                (self.states, np.zeros((self.NUM_DRONES, 16, 1))), axis=2
            )
            self.des_states = np.concatenate(
                (self.des_states, np.zeros((self.NUM_DRONES, 12, 1))), axis=2
            )
            self.controls = np.concatenate(
                (self.controls, np.zeros((self.NUM_DRONES, 12, 1))), axis=2
            )

        self.timestamps[drone, current_counter] = timestamp
        self.states[drone, :, current_counter] = np.hstack(
            [state[0:3], state[10:13], state[7:10], state[13:20]]
        )
        self.des_states[drone, :, current_counter] = des_state
        self.controls[drone, :, current_counter] = control
        self.counters[drone] = current_counter + 1

    ################################################################################
    def save(self):
        """Save numpy version (compressed)."""
        filename = os.path.join(
            self.OUTPUT_FOLDER,
            f"save-flight-{datetime.now().strftime('%m.%d.%Y_%H.%M.%S')}.npz",
        )
        np.savez(
            filename,
            timestamps=self.timestamps,
            states=self.states,
            controls=self.controls,
        )
        print(f"[Logger] Saved binary log to {filename}")

    ################################################################################
    def save_as_csv(self, comment: str = "pid"):
        """Save all drones' data into a single CSV per simulation."""
        folder_name = (
            f"save-flight-{comment}-{datetime.now().strftime('%m.%d.%Y_%H.%M.%S')}"
        )
        csv_path = os.path.join(self.OUTPUT_FOLDER, folder_name)
        os.makedirs(csv_path, exist_ok=True)

        t = np.arange(
            0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ
        )

        for i in range(self.NUM_DRONES):
            n_samples = int(self.counters[i])
            if n_samples == 0:
                continue

            data = {
                "time": t[:n_samples],
                "x": self.states[i, 0, :n_samples],
                "y": self.states[i, 1, :n_samples],
                "z": self.states[i, 2, :n_samples],
                "vx": self.states[i, 3, :n_samples],
                "vy": self.states[i, 4, :n_samples],
                "vz": self.states[i, 5, :n_samples],
                "roll": self.states[i, 6, :n_samples],
                "pitch": self.states[i, 7, :n_samples],
                "yaw": self.states[i, 8, :n_samples],
                "wx": self.states[i, 9, :n_samples],
                "wy": self.states[i, 10, :n_samples],
                "wz": self.states[i, 11, :n_samples],
                "rpm0": self.states[i, 12, :n_samples],
                "rpm1": self.states[i, 13, :n_samples],
                "rpm2": self.states[i, 14, :n_samples],
                "rpm3": self.states[i, 15, :n_samples],
            }

            df = pd.DataFrame(data)
            file_path = os.path.join(csv_path, f"drone{i}_log.csv")
            df.to_csv(file_path, index=False)
            print(f"[Logger] Saved CSV log for drone {i}: {file_path}")

    ################################################################################
    def plot(self, pwm=False):
        """Visualize all logged data with desired vs actual curves."""
        plt.rc(
            "axes",
            prop_cycle=(
                cycler("color", ["r", "g", "b", "y"])
                + cycler("linestyle", ["-", "--", ":", "-."])
            ),
        )
        fig, axs = plt.subplots(10, 2)
        t = np.arange(
            0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ
        )

        for col in range(2):
            for row in range(10):
                axs[row, col].grid(True)

        # Left column: position + attitude
        labels = [
            "x (m)",
            "y (m)",
            "z (m)",
            "r (rad)",
            "p (rad)",
            "y (rad)",
            "wx",
            "wy",
            "wz",
            "time",
        ]
        for i, label in enumerate(labels):
            for j in range(self.NUM_DRONES):
                axs[i, 0].plot(t, self.states[j, i % 12, :], label=f"drone_{j}")
                if i < 9:
                    axs[i, 0].plot(t, self.des_states[j, i % 9, :], linestyle="--")
            axs[i, 0].set_ylabel(label)
        axs[-1, 0].set_xlabel("time (s)")

        # Right column: velocity + motor RPMs
        v_labels = [
            "vx",
            "vy",
            "vz",
            "rdot",
            "pdot",
            "ydot",
            "RPM0",
            "RPM1",
            "RPM2",
            "RPM3",
        ]
        for i, label in enumerate(v_labels):
            for j in range(self.NUM_DRONES):
                if i < 3:
                    axs[i, 1].plot(t, self.states[j, 3 + i, :], label=f"drone_{j}")
                    axs[i, 1].plot(t, self.des_states[j, 3 + i, :], linestyle="--")
                elif i < 6:
                    # RPY rate derivatives
                    rate_idx = 6 + (i - 3)
                    diff = np.hstack(
                        [0, np.diff(self.states[j, rate_idx, :]) * self.LOGGING_FREQ_HZ]
                    )
                    axs[i, 1].plot(t, diff)
                else:
                    axs[i, 1].plot(t, self.states[j, 12 + (i - 6), :])
            axs[i, 1].set_ylabel(label)
        axs[-1, 1].set_xlabel("time (s)")

        fig.subplots_adjust(
            left=0.06, bottom=0.05, right=0.99, top=0.98, wspace=0.15, hspace=0.0
        )

        if self.COLAB:
            plt.savefig(os.path.join("results", "output_figure.png"))
        else:
            plt.show()
