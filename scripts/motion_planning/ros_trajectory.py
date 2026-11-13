"""Convert motion planner output into ROS 2 trajectory messages."""

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from joint_mapping import JOINT_NAMES, math_to_ros


def build_joint_trajectory(trajectory):
    """Build a ROS 2 JointTrajectory from planner samples.

    Each sample:
        (time, Cartesian position, joint positions,
         joint velocities, damping, singularity measure)
    """

    if not trajectory:
        raise ValueError("Trajectory cannot be empty")

    message = JointTrajectory()
    message.joint_names = list(JOINT_NAMES)

    previous_time = -1.0

    for sample in trajectory:
        if len(sample) != 6:
            raise ValueError("Invalid planner sample format")

        time_from_start, _, q, qdot, _, _ = sample

        if time_from_start < 0 or time_from_start <= previous_time:
            if not (time_from_start == 0.0 and previous_time == -1.0):
                raise ValueError("Trajectory timestamps must increase")

        if len(q) != 3 or len(qdot) != 3:
            raise ValueError("Expected exactly three joint values")

        q_ros = math_to_ros(q)
        qdot_ros = math_to_ros(qdot)

        point = JointTrajectoryPoint()
        point.positions = list(q_ros)
        point.velocities = list(qdot_ros)

        seconds = int(time_from_start)
        nanoseconds = int(round(
            (time_from_start - seconds) * 1_000_000_000
        ))

        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000

        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds

        message.points.append(point)
        previous_time = time_from_start

    return message
