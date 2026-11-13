"""Plan and execute Cartesian trajectories through FollowJointTrajectory."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from cartesian_planner import plan_cartesian_line_smooth_with_velocity
from shape_planner import plan_cartesian_square, plan_cartesian_circle
from kinematics import forward_kinematics, check_joint_limits
from ros_trajectory import build_joint_trajectory
from joint_mapping import JOINT_NAMES, ros_to_math
from safety import (
    check_velocity_limits,
    check_acceleration_limits,
)


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__("advanced_trajectory_publisher")

        self.joint_state = None
        self.joint_state_received_at = None
        self.tracking_errors = []

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    def joint_state_callback(self, message):
        self.joint_state = message
        self.joint_state_received_at = time.monotonic()

    def get_current_configuration(self):
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.joint_state is not None:
                positions = dict(zip(
                    self.joint_state.name,
                    self.joint_state.position,
                ))

                if all(name in positions for name in JOINT_NAMES):
                    if time.monotonic() - self.joint_state_received_at > 1.0:
                        raise RuntimeError("Joint feedback is stale")

                    return tuple(positions[name] for name in JOINT_NAMES)

        raise RuntimeError("No complete joint feedback received")

    def validate_samples(self, samples, q_start):
        if len(samples) < 2:
            raise RuntimeError("Trajectory needs at least two samples")

        times = [sample[0] for sample in samples]
        positions = [sample[2] for sample in samples]
        velocities = [sample[3] for sample in samples]

        if any(not math.isfinite(t) for t in times):
            raise RuntimeError("Non-finite trajectory timestamp")

        if times[0] != 0.0:
            raise RuntimeError("Trajectory must start at time zero")

        if any(b <= a for a, b in zip(times, times[1:])):
            raise RuntimeError("Trajectory timestamps must increase")

        # Ensure the planned start matches the live robot configuration.
        start_error = max(
            abs(a - b) for a, b in zip(positions[0], q_start)
        )

        if start_error > 0.01:
            raise RuntimeError(
                f"Planned start differs from live state by "
                f"{start_error:.5f} rad"
            )

        accelerations = []

        for i in range(len(samples)):
            if i == 0:
                dt = times[1] - times[0]
                previous = velocities[0]
                following = velocities[1]
            elif i == len(samples) - 1:
                dt = times[-1] - times[-2]
                previous = velocities[-2]
                following = velocities[-1]
            else:
                dt = times[i + 1] - times[i - 1]
                previous = velocities[i - 1]
                following = velocities[i + 1]

            acceleration = tuple(
                (b - a) / dt
                for a, b in zip(previous, following)
            )
            accelerations.append(acceleration)

        max_velocity = 0.0
        max_acceleration = 0.0

        for q, qdot, qddot in zip(
            positions, velocities, accelerations
        ):
            if not all(
                math.isfinite(value)
                for value in tuple(q) + tuple(qdot) + tuple(qddot)
            ):
                raise RuntimeError("Non-finite trajectory value")

            if not check_joint_limits(q):
                raise RuntimeError("Joint-position limit violation")

            if not check_velocity_limits(qdot):
                raise RuntimeError("Joint-velocity limit violation")

            if not check_acceleration_limits(qddot):
                raise RuntimeError("Joint-acceleration limit violation")

            max_velocity = max(
                max_velocity,
                *(abs(value) for value in qdot),
            )
            max_acceleration = max(
                max_acceleration,
                *(abs(value) for value in qddot),
            )

        print(f"Maximum joint velocity: {max_velocity:.4f} rad/s")
        print(f"Maximum estimated acceleration: {max_acceleration:.4f} rad/s²")
        print(f"Starting configuration difference: {start_error:.6f} rad")

    def execute_trajectory(self, trajectory):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Trajectory action server is unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        print("Sending trajectory goal...")

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Controller rejected the trajectory")

        print("Goal accepted. Executing...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError("No result received from controller")

        print("Controller result code:", result.result.error_code)
        print("Controller message:", result.result.error_string)

        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError("Trajectory execution failed")

        print("SUCCESS: Trajectory completed!")
        self.report_tracking_accuracy()

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        desired = feedback.desired.positions
        actual = feedback.actual.positions

        if len(desired) != 3 or len(actual) != 3:
            return

        desired_tcp = forward_kinematics(ros_to_math(desired))
        actual_tcp = forward_kinematics(ros_to_math(actual))

        error = math.sqrt(sum(
            (a - d) ** 2
            for a, d in zip(actual_tcp, desired_tcp)
        ))

        self.tracking_errors.append(error)

        if len(self.tracking_errors) % 10 == 0:
            print(
                f"TCP tracking error: {error:.4f} mm"
            )

    def report_tracking_accuracy(self):
        if not self.tracking_errors:
            print("No TCP tracking feedback was collected.")
            return

        errors = self.tracking_errors

        rms = math.sqrt(
            sum(error ** 2 for error in errors) / len(errors)
        )

        print("\\n========== TCP ACCURACY REPORT ==========")
        print(f"Feedback samples: {len(errors)}")
        print(f"RMS tracking error: {rms:.4f} mm")
        print(f"Maximum tracking error: {max(errors):.4f} mm")
        print(f"Final positioning error: {errors[-1]:.4f} mm")
        print("=========================================")



def main():
    parser = argparse.ArgumentParser(
        description="Plan and optionally execute safe Cartesian trajectories."
    )
    parser.add_argument(
        "--shape", choices=("line", "square", "circle"), default="line"
    )
    parser.add_argument("--dy", type=float, default=20.0)
    parser.add_argument("--size", type=float, default=20.0)
    parser.add_argument("--radius", type=float, default=10.0)
    parser.add_argument("--blend-radius", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    duration = args.duration
    if duration is None:
        duration = 2.0 if args.shape == "line" else 8.0

    if not math.isfinite(duration) or duration < 2.0:
        parser.error("Duration must be at least 2 seconds")

    if args.shape == "line":
        if not math.isfinite(args.dy) or abs(args.dy) > 20.0:
            parser.error("--dy must be between -20 and 20 mm")
    elif args.shape == "square":
        if not math.isfinite(args.size) or not 0 < args.size <= 20:
            parser.error("--size must be greater than 0 and at most 20 mm")
        if (not math.isfinite(args.blend_radius)
                or not 0 < args.blend_radius < args.size / 2):
            parser.error("--blend-radius must be positive and less than half the square size")
        if duration < 8.0:
            parser.error("Square trajectories require at least 8 seconds")
    else:
        if not math.isfinite(args.radius) or not 0 < args.radius <= 20:
            parser.error("--radius must be greater than 0 and at most 20 mm")
        if duration < 8.0:
            parser.error("Circle trajectories require at least 8 seconds")

    rclpy.init()
    node = TrajectoryPublisher()

    try:
        q_start = node.get_current_configuration()

        if not check_joint_limits(q_start):
            raise RuntimeError("Current configuration violates joint limits")

        start = forward_kinematics(q_start)

        print("\nCurrent TCP:", start)
        print("Selected shape:", args.shape)
        print("Duration:", duration, "seconds")

        if args.shape == "line":
            target = (start[0], start[1] + args.dy, start[2])
            print("Target TCP:", target)

            samples = plan_cartesian_line_smooth_with_velocity(
                start, target,
                duration=duration,
                frequency=50.0,
                elbow="up",
            )

        elif args.shape == "square":
            print("Square side length:", args.size, "mm")
            print("Corner blend radius:", args.blend_radius, "mm")

            samples = plan_cartesian_square(
                start,
                size=args.size,
                duration=duration,
                blend_radius=args.blend_radius,
                frequency=50.0,
                elbow="up",
            )

        else:
            print("Circle radius:", args.radius, "mm")

            samples = plan_cartesian_circle(
                start,
                radius=args.radius,
                duration=duration,
                frequency=50.0,
                elbow="up",
            )

        node.validate_samples(samples, q_start)
        message = build_joint_trajectory(samples)

        print("\nTrajectory validated successfully.")
        print("Joint order:", message.joint_names)
        print("Number of points:", len(message.points))
        print("Duration:", duration, "seconds")

        if args.execute:
            node.execute_trajectory(message)
        else:
            print("\nDRY RUN: No movement command sent.")
            print("All position, velocity and acceleration checks passed.")

    except Exception as error:
        print("\nERROR:", error)
        sys.exit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
