import numpy as np


class RMSETracker:
    def __init__(self):
        self.des_positions = []
        self.real_positions = []

    def add(self, des_pos, real_pos):
        """
        des_pos: np.array shape (3,)
        real_pos: np.array shape (3,)
        """
        self.des_positions.append(des_pos.copy())
        self.real_positions.append(real_pos.copy())

    def compute(self):
        des = np.array(self.des_positions)
        real = np.array(self.real_positions)

        err = des - real
        mse = np.mean(err**2, axis=0)
        rmse = np.sqrt(mse)

        return {
            "rmse_x": float(rmse[0]),
            "rmse_y": float(rmse[1]),
            "rmse_z": float(rmse[2]),
            "rmse_total": float(np.sqrt(np.mean(err**2))),
        }
