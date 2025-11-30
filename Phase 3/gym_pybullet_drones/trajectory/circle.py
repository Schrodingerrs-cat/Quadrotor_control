import numpy as np


# ---------------- Polynomial Basis Matrix ---------------- #
def M(t):
    return np.array(
        [
            [1, t, t**2, t**3, t**4, t**5],
            [0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4],
            [0, 0, 2, 6 * t, 12 * t**2, 20 * t**3],
        ]
    )


def solve_poly(p0, v0, a0, pf, vf, af, T):
    """
    Solve 5th-order polynomial between t = 0 and t = T.
    """
    A = np.vstack((M(0), M(T)))
    b = np.array([p0, v0, a0, pf, vf, af])
    return np.linalg.solve(A, b)


def eval_poly(coeff, t):
    T = np.array([1, t, t * t, t**3, t**4, t**5])
    Td = np.array([0, 1, 2 * t, 3 * t * t, 4 * t**3, 5 * t**4])
    Tdd = np.array([0, 0, 2, 6 * t, 12 * t * t, 20 * t**3])
    return T @ coeff, Td @ coeff, Tdd @ coeff


def circle(t, tf=15.0):
    """
    REQUIRED PHASE-3 TRAJECTORY:
        Phase 1:  0 → tf/3      polynomial hover to (1,0,1)
        Phase 2:  tf/3 → 2tf/3  FULL circle at z=1
        Phase 3:  2tf/3 → tf    polynomial return to hover (0,0,0.5)
        After tf: hold hover
    """

    # ---------------- Phase durations ---------------- #
    t1 = tf / 3.0
    t2 = 2.0 * tf / 3.0
    T1 = t1
    T3 = tf - t2

    # ---------------- Hover after finish ---------------- #
    if t >= tf:
        return {
            "pos": np.array([0, 0, 0.5]),
            "vel": np.zeros(3),
            "acc": np.zeros(3),
            "jerk": np.zeros(3),
            "yaw": 0,
            "yawdot": 0,
        }

    # ------------------- Phase 1 ------------------- #
    if t <= t1:
        start = np.array([0, 0, 0.5])
        end = np.array([1, 0, 1])

        tau = t

        cx = solve_poly(start[0], 0, 0, end[0], 0, 0, T1)
        cy = solve_poly(start[1], 0, 0, end[1], 0, 0, T1)
        cz = solve_poly(start[2], 0, 0, end[2], 0, 0, T1)

        x, vx, ax = eval_poly(cx, tau)
        y, vy, ay = eval_poly(cy, tau)
        z, vz, az = eval_poly(cz, tau)

        return {
            "pos": np.array([x, y, z]),
            "vel": np.array([vx, vy, vz]),
            "acc": np.array([ax, ay, az]),
            "jerk": np.zeros(3),
            "yaw": 0,
            "yawdot": 0,
        }

    # ------------------- Phase 2 (FULL CIRCLE) ------------------- #
    if t1 < t <= t2:
        R = 1.0
        z = 1.0

        omega = 2 * np.pi / (t2 - t1)
        tau = t - t1

        theta = omega * tau  # full 2π

        x = R * np.cos(theta)
        y = R * np.sin(theta)

        vx = -R * omega * np.sin(theta)
        vy = R * omega * np.cos(theta)
        ax = -R * omega**2 * np.cos(theta)
        ay = -R * omega**2 * np.sin(theta)

        return {
            "pos": np.array([x, y, z]),
            "vel": np.array([vx, vy, 0]),
            "acc": np.array([ax, ay, 0]),
            "jerk": np.zeros(3),
            "yaw": 0,
            "yawdot": 0,
        }

    # ------------------- Phase 3 (Return to Hover) ------------------- #
    tau = t - t2

    start = np.array([1.0, 0.0, 1.0])
    end = np.array([0.0, 0.0, 0.5])

    cx = solve_poly(start[0], 0, 0, end[0], 0, 0, T3)
    cy = solve_poly(start[1], 0, 0, end[1], 0, 0, T3)
    cz = solve_poly(start[2], 0, 0, end[2], 0, 0, T3)

    x, vx, ax = eval_poly(cx, tau)
    y, vy, ay = eval_poly(cy, tau)
    z, vz, az = eval_poly(cz, tau)

    return {
        "pos": np.array([x, y, z]),
        "vel": np.array([vx, vy, vz]),
        "acc": np.array([ax, ay, az]),
        "jerk": np.zeros(3),
        "yaw": 0,
        "yawdot": 0,
    }
