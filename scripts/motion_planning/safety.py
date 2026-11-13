"""Joint and trajectory safety limits for the 3-DOF robot arm."""

import math

# Maximum joint velocities from the robot URDF, in rad/s.
JOINT_VELOCITY_LIMITS = (
    2.0,
    2.0,
    2.0,
)

# Acceleration limits in rad/s^2.
# These are configurable planning limits, not verified hardware limits.
JOINT_ACCELERATION_LIMITS = (
    4.0,
    4.0,
    4.0,
)


def check_velocity_limits(qd):
    """Return True if all joint velocities are within configured limits."""
    if len(qd) != 3:
        return False

    return all(
        abs(velocity) <= limit
        for velocity, limit in zip(qd, JOINT_VELOCITY_LIMITS)
    )


def check_acceleration_limits(qdd):
    """Return True if all joint accelerations are within configured limits."""
    if len(qdd) != 3:
        return False

    return all(
        abs(acceleration) <= limit
        for acceleration, limit in zip(qdd, JOINT_ACCELERATION_LIMITS)
    )


def validate_trajectory(positions, velocities, accelerations):
    """Return True if every trajectory sample satisfies all joint limits."""
    if not positions or not velocities or not accelerations:
        return False

    if not (len(positions) == len(velocities) == len(accelerations)):
        return False

    from kinematics import check_joint_limits

    for q, qd, qdd in zip(positions, velocities, accelerations):
        if not check_joint_limits(q):
            return False

        if not check_velocity_limits(qd):
            return False

        if not check_acceleration_limits(qdd):
            return False

    return True


def calculate_speed_scaling(velocities, accelerations):
    """Calculate a time-scaling factor that satisfies joint limits.

    Returns a value between 0 and 1.
    A value of 1 means no slowdown is required.
    """
    if not velocities or not accelerations:
        raise ValueError("Trajectory velocities and accelerations cannot be empty")

    if len(velocities) != len(accelerations):
        raise ValueError("Velocity and acceleration sample counts must match")

    velocity_ratio = 0.0
    acceleration_ratio = 0.0

    for qd, qdd in zip(velocities, accelerations):
        if len(qd) != 3 or len(qdd) != 3:
            raise ValueError("Each sample must contain exactly 3 joints")

        for velocity, limit in zip(qd, JOINT_VELOCITY_LIMITS):
            velocity_ratio = max(
                velocity_ratio,
                abs(velocity) / limit
            )

        for acceleration, limit in zip(qdd, JOINT_ACCELERATION_LIMITS):
            acceleration_ratio = max(
                acceleration_ratio,
                abs(acceleration) / limit
            )

    velocity_scaling = min(1.0, 1.0 / velocity_ratio) if velocity_ratio > 0 else 1.0

    acceleration_scaling = (
        min(1.0, 1.0 / math.sqrt(acceleration_ratio))
        if acceleration_ratio > 0 else 1.0
    )

    return min(velocity_scaling, acceleration_scaling)


def make_trajectory_safe(times, positions, velocities, accelerations):
    """Automatically scale a trajectory to satisfy joint limits."""
    from trajectories import apply_time_scaling

    scaling = calculate_speed_scaling(velocities, accelerations)

    result = apply_time_scaling(
        times,
        positions,
        velocities,
        accelerations,
        scaling,
    )

    scaled_times, scaled_positions, scaled_velocities, scaled_accelerations = result

    if not validate_trajectory(
        scaled_positions,
        scaled_velocities,
        scaled_accelerations,
    ):
        raise ValueError("Trajectory violates joint limits after scaling")

    return (
        scaled_times,
        scaled_positions,
        scaled_velocities,
        scaled_accelerations,
        scaling,
    )
