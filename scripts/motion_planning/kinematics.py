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


def check_singularity(
    q,
    tolerance=1e-6,
    position_tolerance_mm=1e-3,
    angular_tolerance=1e-6,
):
    """Return True for singular or near-singular configurations."""
    conditions = analytical_singularity_conditions(
        q,
        position_tolerance_mm=position_tolerance_mm,
        angular_tolerance=angular_tolerance,
    )
    return (
        singularity_measure(q) <= tolerance
        or conditions["singular"]
    )


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


# ---------------------------------------------------------------------------
# Analytical Jacobian from the supplied 3-DOF derivation.
#
# analytical convention:
#   theta1 = q1
#   theta2 = PHI1_0 - q2 - pi/2
#   theta3 = PHI2_0 - PHI1_0 - q3
#
# analytical L2 = robot L1; analytical L3 = robot L2.
# All translational quantities are in mm.
# ---------------------------------------------------------------------------

ANALYTICAL_L2 = L1
ANALYTICAL_L3 = L2


def analytical_joint_angles(q):
    """Convert this robot's joint angles to the analytical's convention."""
    q1, q2, q3 = q
    theta1 = q1
    theta2 = PHI1_0 - q2 - math.pi / 2.0
    theta3 = PHI2_0 - PHI1_0 - q3
    return theta1, theta2, theta3


def _analytical_terms(q):
    """Calculate A, B and the trigonometric terms in the supplied derivation."""
    theta1, theta2, theta3 = analytical_joint_angles(q)
    theta23 = theta2 + theta3

    s1, c1 = math.sin(theta1), math.cos(theta1)
    s2, c2 = math.sin(theta2), math.cos(theta2)
    s23, c23 = math.sin(theta23), math.cos(theta23)
    s3 = math.sin(theta3)

    A = ANALYTICAL_L2 * s2 + ANALYTICAL_L3 * s23
    B = ANALYTICAL_L2 * c2 + ANALYTICAL_L3 * c23

    return theta1, theta2, theta3, A, B, s1, c1, s2, c2, s23, c23, s3


def analytical_forward_kinematics_transform(q):
    """Return the exact 4x4 homogeneous transform from the analytical."""
    (_, theta2, theta3, A, B, s1, c1,
     s2, c2, s23, c23, _) = _analytical_terms(q)

    return [
        [-c1 * s23, -c1 * c23, s1, -c1 * A],
        [-s1 * s23, -s1 * c23, -c1, -s1 * A],
        [c23, -s23, 0.0, SHOULDER[2] + B],
        [0.0, 0.0, 0.0, 1.0],
    ]


def analytical_full_jacobian(q):
    """Return the exact 6x3 geometric Jacobian shown in the analytical.

    Its columns map theta_dot to [vx, vy, vz, wx, wy, wz].
    """
    (_, _, _, A, B, s1, c1,
     _, _, s23, c23, _) = _analytical_terms(q)

    a = ANALYTICAL_L3 * c23
    b = ANALYTICAL_L3 * s23

    return [
        [s1 * A, -c1 * B, -c1 * a],
        [-c1 * A, -s1 * B, -s1 * a],
        [0.0, -A, -b],
        [0.0, s1, s1],
        [0.0, -c1, -c1],
        [1.0, 0.0, 0.0],
    ]


def analytical_linear_jacobian(q):
    """Return the top 3x3 translational part of the analytical Jacobian."""
    return [row[:] for row in analytical_full_jacobian(q)[:3]]


def analytical_jacobian_determinant(q):
    """Return det(Jv) = A * L2 * L3 * sin(theta3)."""
    (_, _, _, A, _, _, _, _, _, _, _, s3) = _analytical_terms(q)
    return A * ANALYTICAL_L2 * ANALYTICAL_L3 * s3


def analytical_singularity_conditions(
    q,
    position_tolerance_mm=1e-3,
    angular_tolerance=1e-6,
):
    """Report base-axis and planar-elbow singularity conditions.

    A is measured in millimetres; sin(theta3) is dimensionless.
    Their tolerances must therefore be specified independently.
    """
    (_, _, _, A, _, _, _, _, _, _, _, s3) = _analytical_terms(q)
    A_zero = abs(A) <= position_tolerance_mm
    sin_theta3_zero = abs(s3) <= angular_tolerance
    return {
        "A_zero": A_zero,
        "sin_theta3_zero": sin_theta3_zero,
        "singular": A_zero or sin_theta3_zero,
    }


def analytical_inverse_jacobian(q, tolerance=1e-12):
    """Return the exact analytical Jv inverse from analytical Step 8.

    Maps [vx, vy, vz] to [theta1_dot, theta2_dot, theta3_dot].
    Raises ValueError at a singular or nearly singular configuration.
    """
    (_, _, _, A, B, s1, c1,
     _, _, s23, c23, s3) = _analytical_terms(q)

    denominator = ANALYTICAL_L2 * ANALYTICAL_L3 * s3

    if abs(A) <= tolerance or abs(denominator) <= tolerance:
        raise ValueError(
            "Analytical inverse Jacobian is singular at this configuration"
        )

    return [
        [s1 / A, -c1 / A, 0.0],
        [
            -c1 * s23 / (ANALYTICAL_L2 * s3),
            -s1 * s23 / (ANALYTICAL_L2 * s3),
            c23 / (ANALYTICAL_L2 * s3),
        ],
        [
            c1 * A / denominator,
            s1 * A / denominator,
            -B / denominator,
        ],
    ]


def analytical_inverse_joint_velocity(q, cartesian_velocity):
    """Map Cartesian velocity to this robot's q_dot using the exact inverse.

    The analytical inverse returns theta_dot. This function converts it to
    the robot's q_dot convention, where q2_dot=-theta2_dot and
    q3_dot=-theta3_dot.
    """
    if len(cartesian_velocity) != 3:
        raise ValueError("Cartesian velocity must contain vx, vy, vz")

    inverse = analytical_inverse_jacobian(q)
    theta_dot = [
        sum(inverse[i][j] * cartesian_velocity[j] for j in range(3))
        for i in range(3)
    ]

    return theta_dot[0], -theta_dot[1], -theta_dot[2]
