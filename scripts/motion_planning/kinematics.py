"""Analytical kinematics for the 3-DOF robot arm.

Coordinates and link lengths are in millimetres.
Joint angles are in radians.
"""

import math


# Geometry extracted from the robot URDF.
SHOULDER = (0.0, 0.0, 52.0)
ELBOW = (30.0, 0.0, 288.8)
TIP = (378.0, 0.0, 293.0)

L1 = math.hypot(30.0, 236.8)
L2 = math.hypot(348.0, 4.2)

PHI1_0 = math.atan2(236.8, 30.0)
PHI2_0 = math.atan2(4.2, 348.0)

JOINT_LIMITS = (
    (-3.14, 3.14),
    (-1.57, 1.57),
    (-1.57, 1.57),
)


def forward_kinematics(q):
    """Return end-effector (x, y, z) in millimetres."""
    q1, q2, q3 = q

    phi1 = PHI1_0 - q2
    phi2 = PHI2_0 - q2 - q3

    r = L1 * math.cos(phi1) + L2 * math.cos(phi2)

    x = r * math.cos(q1)
    y = r * math.sin(q1)
    z = SHOULDER[2] + L1 * math.sin(phi1) + L2 * math.sin(phi2)

    return x, y, z


def inverse_kinematics(x, y, z, elbow="up", enforce_limits=False):
    """Return joint angles for an XYZ target.

    elbow="up" or elbow="down" selects the planar elbow configuration.
    The negative planar elbow angle corresponds to elbow-up.
    """
    if elbow not in ("up", "down"):
        raise ValueError("elbow must be 'up' or 'down'")

    r = math.hypot(x, y)
    q1 = math.atan2(y, x)
    zt = z - SHOULDER[2]

    D = (
        r**2 + zt**2 - L1**2 - L2**2
    ) / (2.0 * L1 * L2)

    tolerance = 1e-9

    if D < -1.0 - tolerance or D > 1.0 + tolerance:
        raise ValueError("Target is outside the robot workspace")

    D = max(-1.0, min(1.0, D))

    if elbow == "up":
        delta = -math.acos(D)
    else:
        delta = math.acos(D)

    phi1 = math.atan2(zt, r) - math.atan2(
        L2 * math.sin(delta),
        L1 + L2 * math.cos(delta)
    )

    phi2 = phi1 + delta

    q2 = PHI1_0 - phi1
    q3 = (PHI2_0 - phi2) - q2

    solution = (q1, q2, q3)

    if enforce_limits and not check_joint_limits(solution):
        raise ValueError("IK solution violates robot joint limits")

    return solution


def jacobian(q):
    """Return the 3x3 Jacobian mapping joint velocity to XYZ velocity."""
    q1, q2, q3 = q

    phi1 = PHI1_0 - q2
    phi2 = PHI2_0 - q2 - q3

    r = L1 * math.cos(phi1) + L2 * math.cos(phi2)

    dr2 = L1 * math.sin(phi1) + L2 * math.sin(phi2)
    dr3 = L2 * math.sin(phi2)

    return [
        [
            -r * math.sin(q1),
            dr2 * math.cos(q1),
            dr3 * math.cos(q1),
        ],
        [
            r * math.cos(q1),
            dr2 * math.sin(q1),
            dr3 * math.sin(q1),
        ],
        [
            0.0,
            -r,
            -L2 * math.cos(phi2),
        ],
    ]


def check_joint_limits(q):
    """Return True if all joint angles are within URDF limits."""
    if len(q) != 3:
        return False

    return all(
        lower <= angle <= upper
        for angle, (lower, upper) in zip(q, JOINT_LIMITS)
    )


def singularity_measure(q):
    """Return a dimensionless singularity measure between 0 and 1.

    Values close to zero indicate a singular or nearly singular
    linear Jacobian. A value of 1 indicates mutually orthogonal
    Jacobian columns.
    """
    j = jacobian(q)

    columns = [
        [j[row][col] for row in range(3)]
        for col in range(3)
    ]

    norms = [
        math.sqrt(sum(value * value for value in column))
        for column in columns
    ]

    if any(norm == 0.0 for norm in norms):
        return 0.0

    determinant = (
        j[0][0] * (j[1][1]*j[2][2] - j[1][2]*j[2][1])
        - j[0][1] * (j[1][0]*j[2][2] - j[1][2]*j[2][0])
        + j[0][2] * (j[1][0]*j[2][1] - j[1][1]*j[2][0])
    )

    return abs(determinant) / (norms[0] * norms[1] * norms[2])


def check_singularity(q, tolerance=1e-6):
    """Return True when the linear Jacobian is singular or nearly singular."""
    return singularity_measure(q) <= tolerance


def _transpose(matrix):
    """Return the transpose of a matrix."""
    return [list(row) for row in zip(*matrix)]


def _matrix_multiply(a, b):
    """Multiply two matrices."""
    return [
        [
            sum(a[i][k] * b[k][j] for k in range(len(b)))
            for j in range(len(b[0]))
        ]
        for i in range(len(a))
    ]


def _inverse_3x3(matrix):
    """Calculate the inverse of a nonsingular 3x3 matrix."""
    augmented = [
        list(matrix[i]) + [
            1.0 if i == j else 0.0 for j in range(3)
        ]
        for i in range(3)
    ]

    for col in range(3):
        pivot = max(
            range(col, 3),
            key=lambda row: abs(augmented[row][col])
        )

        if abs(augmented[pivot][col]) < 1e-15:
            raise ValueError("Matrix is singular")

        augmented[col], augmented[pivot] = (
            augmented[pivot], augmented[col]
        )

        divisor = augmented[col][col]
        augmented[col] = [
            value / divisor for value in augmented[col]
        ]

        for row in range(3):
            if row != col:
                factor = augmented[row][col]
                augmented[row] = [
                    augmented[row][i] - factor * augmented[col][i]
                    for i in range(6)
                ]

    return [row[3:] for row in augmented]


def inverse_jacobian(q, damping=1.0):
    """Return the damped inverse of the 3x3 linear Jacobian.

    Maps Cartesian velocity (mm/s) to joint velocity (rad/s).

    Damped Least Squares:
        J_damped_inverse = (J.T @ J + damping^2 * I)^-1 @ J.T

    Damping is expressed in the same scale as the linear Jacobian
    (mm/rad). A positive damping value improves stability near
    singular configurations.
    """
    if damping <= 0.0:
        raise ValueError("Damping must be positive")

    j = jacobian(q)
    jt = _transpose(j)

    normal = _matrix_multiply(jt, j)

    for i in range(3):
        normal[i][i] += damping ** 2

    return _matrix_multiply(_inverse_3x3(normal), jt)
