import argparse
import csv
import os
import subprocess

RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)
SUMMARY_CSV = os.path.join(RESULTS_DIR, "experiment_summary.csv")

# Experiments: (ExpID, GainMode, Trajectory)
EXPERIMENTS = [
    # Baseline / gain variation experiments
    ("A", "baseline", "circle"),
    ("A", "baseline", "diamond"),
    ("B", "low_kd", "circle"),
    ("B", "low_kd", "diamond"),
    ("C", "low_kp", "circle"),
    ("C", "low_kp", "diamond"),
    # Duration variation experiments (all baseline gains)
    ("D1", "baseline", "circle"),
    ("D2", "baseline", "circle"),
    ("D3", "baseline", "circle"),
    ("D4", "baseline", "circle"),
    ("D5", "baseline", "circle"),
    ("D6", "baseline", "circle"),
    ("D1", "baseline", "diamond"),
    ("D2", "baseline", "diamond"),
    ("D3", "baseline", "diamond"),
    ("D4", "baseline", "diamond"),
    ("D5", "baseline", "diamond"),
    ("D6", "baseline", "diamond"),
]

# Map "ExpID_trajectory" → (ExpID, mode, traj)
ID_MAP = {f"{exp}_{traj}": (exp, mode, traj) for (exp, mode, traj) in EXPERIMENTS}

# Duration map for D* experiments
TF_MAP = {
    "circle": {
        "D1": 8,
        "D2": 10,
        "D3": 12,
        "D4": 15,
        "D5": 18,
        "D6": 20,
    },
    "diamond": {
        "D1": 6,
        "D2": 8,
        "D3": 10,
        "D4": 12,
        "D5": 15,
        "D6": 18,
    },
}


def get_tf(exp_id: str, traj: str) -> float:
    """
    Return total duration tf for a given experiment and trajectory.

    For A/B/C:
        circle  -> 15
        diamond -> 8

    For D1–D6:
        use TF_MAP.
    """
    if exp_id.startswith("D"):
        # D1..D6
        return TF_MAP[traj][exp_id]

    # A/B/C default durations
    if traj == "circle":
        return 15.0
    else:  # "diamond"
        return 8.0


def pkg_root():
    """Find Phase_3/gym_pybullet_drones path and set PYTHONPATH root."""
    here = os.path.abspath(
        os.path.dirname(__file__)
    )  # .../gym_pybullet_drones/examples
    gym_root = os.path.abspath(os.path.join(here, ".."))  # .../gym_pybullet_drones
    phase_root = os.path.abspath(os.path.join(gym_root, ".."))  # .../Phase_3
    return phase_root


def env_with_pythonpath():
    """Return environment with PYTHONPATH including Phase_3 root."""
    env = os.environ.copy()
    env["PYTHONPATH"] = pkg_root()
    return env


def run_single(id_key: str):
    """Run a single experiment with GUI, e.g. A_circle, D3_diamond."""
    if id_key not in ID_MAP:
        print(f"Invalid experiment ID: {id_key}")
        print("Valid IDs:", list(ID_MAP.keys()))
        return

    exp, mode, traj = ID_MAP[id_key]
    tf = get_tf(exp, traj)

    print(f"\n===== Running SINGLE EXPERIMENT: {exp} | {mode} | {traj} | tf={tf} =====")

    cmd = [
        "python3",
        "trajectory_tracking.py",
        f"--exp_mode={mode}",
        f"--traj={traj}",
        f"--tf={tf}",
        "--gui=true",
        "--batch=false",
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env_with_pythonpath(),
        cwd=os.path.dirname(__file__),  # ensure working dir = examples/
    )

    for line in proc.stdout:
        print(line, end="")
    proc.wait()

    print("\n===== DONE =====")


def run_batch():
    """Run all experiments without GUI and write RMSE summary CSV."""
    print("\n===== Running ALL Experiments (Batch Mode) =====\n")

    with open(SUMMARY_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "Experiment",
                "GainMode",
                "Trajectory",
                "tf",
                "RMSE_X",
                "RMSE_Y",
                "RMSE_Z",
                "RMSE_Total",
            ]
        )

        for exp, mode, traj in EXPERIMENTS:
            tf = get_tf(exp, traj)
            print(
                f"\n===== Running EXP {exp} | mode={mode} | traj={traj} | tf={tf} ====="
            )

            cmd = [
                "python3",
                "trajectory_tracking.py",
                f"--exp_mode={mode}",
                f"--traj={traj}",
                f"--tf={tf}",
                "--gui=false",
                "--batch=true",
            ]

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env_with_pythonpath(),
                cwd=os.path.dirname(__file__),
            )

            rmse_x = rmse_y = rmse_z = rmse_tot = None

            for line in proc.stdout:
                print(line, end="")
                if "RMSE_X:" in line:
                    rmse_x = float(line.split(":")[1].replace("m", "").strip())
                if "RMSE_Y:" in line:
                    rmse_y = float(line.split(":")[1].replace("m", "").strip())
                if "RMSE_Z:" in line:
                    rmse_z = float(line.split(":")[1].replace("m", "").strip())
                if "TOTAL RMSE:" in line:
                    rmse_tot = float(line.split(":")[1].replace("m", "").strip())

            proc.wait()

            if None in [rmse_x, rmse_y, rmse_z, rmse_tot]:
                print("ERROR: RMSE NOT PARSED")
            else:
                writer.writerow([exp, mode, traj, tf, rmse_x, rmse_y, rmse_z, rmse_tot])

    print("\n===== ALL EXPERIMENTS COMPLETE =====")
    print(f"Results saved to {SUMMARY_CSV}")


def main(single_exp: str | None = None):
    if single_exp:
        run_single(single_exp)
    else:
        run_batch()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--single",
        type=str,
        default=None,
        help="Run only a single experiment, e.g. A_circle, B_diamond, D3_circle",
    )
    args = parser.parse_args()
    main(single_exp=args.single)
