"""Quintic trajectory generation for robot joint movements.

Positions are in radians.
Time is in seconds.
Velocity is in radians/second.
Acceleration is in radians/second squared.
"""


def quintic_trajectory(q_start, q_end, duration, t):
    """Calculate joint position, velocity and acceleration at time t.

    Uses a quintic polynomial with zero endpoint velocity
    and acceleration.
    """

    if len(q_start) != len(q_end):
        raise ValueError("Start and end configurations must match")

    if duration <= 0.0:
        raise ValueError("Duration must be positive")

    if t < 0.0 or t > duration:
        raise ValueError("Time must be between 0 and duration")

    u = t / duration

    s = 10*u**3 - 15*u**4 + 6*u**5

    ds = 30*u**2 - 60*u**3 + 30*u**4

    dds = 60*u - 180*u**2 + 120*u**3

    positions = []
    velocities = []
    accelerations = []

    for start, end in zip(q_start, q_end):

        delta = end - start

        position = start + delta * s
        velocity = delta * ds / duration
        acceleration = delta * dds / duration**2

        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)

    return positions, velocities, accelerations


def apply_time_scaling(times, positions, velocities, accelerations, scaling):
    """Apply time scaling to an existing sampled trajectory.

    Positions remain unchanged.
    Times and duration increase by 1/scaling.
    Velocities scale by scaling.
    Accelerations scale by scaling squared.
    """
    if not (len(times) == len(positions) == len(velocities) == len(accelerations)):
        raise ValueError("All trajectory arrays must have equal lengths")

    if not times:
        raise ValueError("Trajectory cannot be empty")

    if scaling <= 0.0 or scaling > 1.0:
        raise ValueError("Scaling must be greater than 0 and at most 1")

    start_time = times[0]
    scaled_times = [start_time + (t - start_time) / scaling for t in times]

    scaled_velocities = [
        [value * scaling for value in qd]
        for qd in velocities
    ]

    scaled_accelerations = [
        [value * scaling**2 for value in qdd]
        for qdd in accelerations
    ]

    return (
        scaled_times,
        [list(q) for q in positions],
        scaled_velocities,
        scaled_accelerations,
    )
