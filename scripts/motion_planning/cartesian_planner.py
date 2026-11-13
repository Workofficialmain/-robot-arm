"""Cartesian path planning for the 3-DOF robot arm.

Cartesian positions are in millimetres.
Joint angles are in radians.
Time is in seconds.
"""

import math

from kinematics import (
    inverse_kinematics,
    check_joint_limits,
    inverse_jacobian,
    singularity_measure,
)


def plan_cartesian_line(
    start,
    end,
    duration,
    frequency=50.0,
    elbow="up",
):
    """Generate a straight Cartesian path using inverse kinematics.

    Returns:
        (time, Cartesian position, joint configuration)
    """

    if len(start) != 3 or len(end) != 3:
        raise ValueError("Start and end must contain XYZ coordinates")

    if duration <= 0.0:
        raise ValueError("Duration must be positive")

    if frequency <= 0.0:
        raise ValueError("Frequency must be positive")

    if elbow not in ("up", "down"):
        raise ValueError("Elbow must be 'up' or 'down'")

    sample_count = math.ceil(duration * frequency)

    path = []

    for i in range(sample_count + 1):
        t = duration * i / sample_count

        u = t / duration

        position = tuple(
            a + u * (b - a)
            for a, b in zip(start, end)
        )

        q = inverse_kinematics(
            position[0],
            position[1],
            position[2],
            elbow=elbow,
            enforce_limits=True,
        )

        if not check_joint_limits(q):
            raise ValueError("Planned path violates joint limits")

        path.append((t, position, q))

    return path


def plan_cartesian_line_with_velocity(
    start,
    end,
    duration,
    frequency=50.0,
    elbow="up",
):
    """Generate Cartesian positions and DLS joint velocities.

    Returns:
        (time, Cartesian position, joint positions,
         joint velocities, damping, singularity measure)

    Cartesian velocity is constant along the line.
    """

    path = plan_cartesian_line(
        start,
        end,
        duration,
        frequency,
        elbow,
    )

    cartesian_velocity = [
        (b - a) / duration
        for a, b in zip(start, end)
    ]

    velocity_path = []

    for t, position, q in path:

        measure = singularity_measure(q)

        # Increase damping as the robot approaches a singularity.
        damping = 0.1 + 5.0 * max(
            0.0,
            1.0 - measure / 0.1,
        )

        j_inverse = inverse_jacobian(
            q,
            damping=damping,
        )

        joint_velocity = [
            sum(
                j_inverse[row][col] * cartesian_velocity[col]
                for col in range(3)
            )
            for row in range(3)
        ]

        velocity_path.append(
            (
                t,
                position,
                q,
                tuple(joint_velocity),
                damping,
                measure,
            )
        )

    return velocity_path


def plan_cartesian_line_smooth_with_velocity(
    start,
    end,
    duration,
    frequency=50.0,
    elbow="up",
):
    """Generate a straight Cartesian path with smooth start and stop.

    Uses quintic time scaling.

    Returns:
        (time, Cartesian position, joint positions,
         joint velocities, damping, singularity measure)
    """

    if len(start) != 3 or len(end) != 3:
        raise ValueError("Start and end must contain XYZ coordinates")

    if duration <= 0.0:
        raise ValueError("Duration must be positive")

    if frequency <= 0.0:
        raise ValueError("Frequency must be positive")

    if elbow not in ("up", "down"):
        raise ValueError("Elbow must be 'up' or 'down'")

    sample_count = math.ceil(duration * frequency)

    velocity_path = []

    for i in range(sample_count + 1):

        t = duration * i / sample_count
        u = t / duration

        s = 10*u**3 - 15*u**4 + 6*u**5

        ds = (
            30*u**2 - 60*u**3 + 30*u**4
        ) / duration

        position = tuple(
            a + s * (b - a)
            for a, b in zip(start, end)
        )

        cartesian_velocity = [
            ds * (b - a)
            for a, b in zip(start, end)
        ]

        q = inverse_kinematics(
            position[0],
            position[1],
            position[2],
            elbow=elbow,
            enforce_limits=True,
        )

        if not check_joint_limits(q):
            raise ValueError("Planned path violates joint limits")

        measure = singularity_measure(q)

        damping = 0.1 + 5.0 * max(
            0.0,
            1.0 - measure / 0.1,
        )

        j_inverse = inverse_jacobian(
            q,
            damping=damping,
        )

        joint_velocity = [
            sum(
                j_inverse[row][col] * cartesian_velocity[col]
                for col in range(3)
            )
            for row in range(3)
        ]

        velocity_path.append(
            (
                t,
                position,
                q,
                tuple(joint_velocity),
                damping,
                measure,
            )
        )

    return velocity_path
