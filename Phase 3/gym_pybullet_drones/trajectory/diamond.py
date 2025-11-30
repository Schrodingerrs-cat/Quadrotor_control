import numpy as np


# --- Polynomial basis --- #
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
    Correct version: solve polynomial only over [0, T].
    No t0, tf confusion.
    """
    A = np.vstack((M(0), M(T)))
    b = np.array([p0, v0, a0, pf, vf, af])
    return np.linalg.solve(A, b)


def eval_poly(coeff, tau):
    T = np.array([1, tau, tau**2, tau**3, tau**4, tau**5])
    Td = np.array([0, 1, 2 * tau, 3 * tau**2, 4 * tau**3, 5 * tau**4])
    Tdd = np.array([0, 0, 2, 6 * tau, 12 * tau**2, 20 * tau**3])
    return T @ coeff, Td @ coeff, Tdd @ coeff


def diamond(t, tfinal=8.0, z0=0.5):
    """
    Diamond trajectory with 4 polynomial segments in (y,z)
    and linear motion in x.
    """

    # After finishing the diamond → hover
    if t >= tfinal:
        return {
            "pos": np.array([1.0, 0.0, z0]),
            "vel": np.zeros(3),
            "acc": np.zeros(3),
            "jerk": np.zeros(3),
            "yaw": 0,
            "yawdot": 0,
        }

    # Diamond waypoints (relative y,z)
    u = 1 / np.sqrt(2)
    p = [
        np.array([0, 0]),
        np.array([u, u]),
        np.array([0, 2 * u]),
        np.array([-u, u]),
        np.array([0, 0]),
    ]

    # Segment duration
    segT = tfinal / 4

    # Identify segment index
    seg = int(t // segT)
    seg = min(seg, 3)

    # local time in segment
    tau = t - seg * segT

    # segment start & end
    p0 = p[seg]
    p1 = p[seg + 1]

    # --- Y polynomial ---
    cy = solve_poly(p0[0], 0, 0, p1[0], 0, 0, segT)
    y, vy, ay = eval_poly(cy, tau)

    # --- Z polynomial ---
    cz = solve_poly(p0[1], 0, 0, p1[1], 0, 0, segT)
    z_rel, vz, az = eval_poly(cz, tau)
    z = z0 + z_rel

    # --- X linear motion ---
    x = t / tfinal
    vx = 1 / tfinal
    ax = 0

    pos = np.array([x, y, z])
    vel = np.array([vx, vy, vz])
    acc = np.array([ax, ay, az])

    return {
        "pos": pos,
        "vel": vel,
        "acc": acc,
        "jerk": np.zeros(3),
        "yaw": 0,
        "yawdot": 0,
    }
