"""Mapping between mathematical joint angles and ROS 2 joint positions.

Joint angles are expressed in radians.

The mathematical joint order matches the ROS 2 controller order.
Offsets and direction signs are defined from the URDF convention.
"""

JOINT_NAMES = (
    "joint_base_rotate",
    "joint_shoulder",
    "joint_elbow",
)

# Mathematical angle offsets in radians.
JOINT_OFFSETS = (
    0.0,
    0.0,
    0.0,
)

# Direction signs: +1 means the mathematical and ROS directions match.
JOINT_SIGNS = (
    1.0,
    1.0,
    1.0,
)


def math_to_ros(q_math):
    """Convert mathematical joint angles to ROS 2 joint positions."""
    if len(q_math) != 3:
        raise ValueError("Expected exactly three joint angles")

    return tuple(
        sign * angle + offset
        for angle, sign, offset in zip(
            q_math, JOINT_SIGNS, JOINT_OFFSETS
        )
    )


def ros_to_math(q_ros):
    """Convert ROS 2 joint positions to mathematical joint angles."""
    if len(q_ros) != 3:
        raise ValueError("Expected exactly three joint positions")

    return tuple(
        (position - offset) / sign
        for position, sign, offset in zip(
            q_ros, JOINT_SIGNS, JOINT_OFFSETS
        )
    )
