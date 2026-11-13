"""Blended square and smooth circle Cartesian trajectory planners."""

import math

from kinematics import (
    inverse_kinematics,
    inverse_jacobian,
    check_joint_limits,
    singularity_measure,
)


def _line_segment(a, b):
    length = math.dist(a, b)

    def evaluate(s):
        u = min(1.0, max(0.0, s / length))
        position = tuple(x + u * (y - x) for x, y in zip(a, b))
        tangent = tuple((y - x) / length for x, y in zip(a, b))
        return position, tangent

    return length, evaluate


def _arc_segment(center, radius, start_angle, end_angle, z):
    length = radius * abs(end_angle - start_angle)
    direction = 1.0 if end_angle > start_angle else -1.0

    def evaluate(s):
        angle = start_angle + direction * min(
            abs(end_angle - start_angle), s / radius
        )
        position = (
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle),
            z,
        )
        tangent = (
            -direction * math.sin(angle),
            direction * math.cos(angle),
            0.0,
        )
        return position, tangent

    return length, evaluate


def _rounded_square_path(start, size, radius):
    """Create a closed square with four rounded corners.

    The path begins and ends at the supplied TCP position.
    """
    x, y, z = start

    if size <= 0 or radius <= 0 or radius >= size / 2:
        raise ValueError("Require size > 0 and 0 < radius < size/2")

    segments = []

    # Bottom edge, beginning at the current TCP.
    segments.append(_line_segment(
        (x, y, z),
        (x + size - 2 * radius, y, z)
    ))

    # Bottom-right rounded corner.
    segments.append(_arc_segment(
        (x + size - 2 * radius, y + radius),
        radius, -math.pi / 2, 0, z
    ))

    # Right edge.
    segments.append(_line_segment(
        (x + size - radius, y + radius, z),
        (x + size - radius, y + size - radius, z)
    ))

    # Top-right rounded corner.
    segments.append(_arc_segment(
        (x + size - 2 * radius, y + size - radius),
        radius, 0, math.pi / 2, z
    ))

    # Top edge.
    segments.append(_line_segment(
        (x + size - 2 * radius, y + size, z),
        (x, y + size, z)
    ))

    # Top-left rounded corner.
    segments.append(_arc_segment(
        (x, y + size - radius),
        radius, math.pi / 2, math.pi, z
    ))

    # Left edge.
    segments.append(_line_segment(
        (x - radius, y + size - radius, z),
        (x - radius, y + radius, z)
    ))

    # Bottom-left rounded corner returns to the starting TCP.
    segments.append(_arc_segment(
        (x, y + radius),
        radius, math.pi, 3 * math.pi / 2, z
    ))

    return segments


def _plan_path(segments, duration, frequency, elbow):
    if duration <= 0 or frequency <= 0:
        raise ValueError("Duration and frequency must be positive")

    if elbow not in ("up", "down"):
        raise ValueError("Elbow must be 'up' or 'down'")

    lengths = [segment[0] for segment in segments]
    total_length = sum(lengths)

    if total_length <= 0:
        raise ValueError("Path length must be positive")

    count = int(round(duration * frequency))
    samples = []
    cumulative = 0.0

    for i in range(count + 1):
        t = duration * i / count
        u = t / duration

        # Quintic time scaling: zero speed at both ends.
        blend = 10*u**3 - 15*u**4 + 6*u**5
        blend_rate = (30*u**2 - 60*u**3 + 30*u**4) / duration

        distance = total_length * blend
        cartesian_speed = total_length * blend_rate

        remaining = distance
        position, tangent = None, None

        for length, evaluate in segments:
            if remaining <= length:
                position, tangent = evaluate(remaining)
                break
            remaining -= length

        if position is None:
            position, tangent = segments[-1][1](lengths[-1])

        velocity = tuple(v * cartesian_speed for v in tangent)

        q = inverse_kinematics(
            *position, elbow=elbow, enforce_limits=True
        )

        if not check_joint_limits(q):
            raise ValueError("Joint-limit violation")

        measure = singularity_measure(q)
        damping = 0.1 + 5.0 * max(0.0, 1.0 - measure / 0.1)

        inverse = inverse_jacobian(q, damping=damping)
        qdot = tuple(
            sum(inverse[row][col] * velocity[col] for col in range(3))
            for row in range(3)
        )

        samples.append((
            t, position, q, qdot, damping, measure
        ))

        cumulative += 0.0

    return samples


def plan_cartesian_square(start, size, duration, blend_radius=10.0,
                          frequency=50.0, elbow="up"):
    """Plan a rounded square in the XY plane, starting at the TCP."""
    segments = _rounded_square_path(start, size, blend_radius)
    return _plan_path(segments, duration, frequency, elbow)


def plan_cartesian_circle(start, radius, duration,
                          frequency=50.0, elbow="up"):
    """Plan a circle in the XY plane, starting at the TCP."""
    if radius <= 0:
        raise ValueError("Circle radius must be positive")

    x, y, z = start
    center = (x, y - radius)

    segments = [
        _arc_segment(center, radius, math.pi / 2, 5 * math.pi / 2, z)
    ]

    return _plan_path(segments, duration, frequency, elbow)
