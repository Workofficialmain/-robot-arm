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
from gazebo_msgs.srv import SpawnEntity

from cartesian_planner import plan_cartesian_line_smooth_with_velocity
from shape_planner import (
    plan_cartesian_square,
    plan_cartesian_circle,
    _arc_segment,
    _plan_path,
)
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
        self.measured_actual_tcp = []      # (x_w, y_w, z_w) in meters, actual measured coordinates from /joint_states + FK
        self.collecting_trail = False
        self.trail_chunk_count = 0
        self.last_spawned_index = 0
        self.chunk_dist_threshold = 0.035  # 35 mm traveled per chunk (~10 chunks total for 377 mm circle)
        self.spawn_future = None

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

        self.spawn_client = self.create_client(
            SpawnEntity,
            "/spawn_entity",
        )

    def joint_state_callback(self, message):
        self.joint_state = message
        self.joint_state_received_at = time.monotonic()

        if self.collecting_trail:
            positions = dict(zip(message.name, message.position))
            if all(name in positions for name in JOINT_NAMES):
                q_ros = tuple(positions[name] for name in JOINT_NAMES)
                q_math = ros_to_math(q_ros)
                tcp_cad = forward_kinematics(q_math)
                # World coordinates in meters (applying verified +80mm Gazebo base lift)
                x_w = tcp_cad[0] / 1000.0
                y_w = tcp_cad[1] / 1000.0
                z_w = (tcp_cad[2] + 80.0) / 1000.0

                self.measured_actual_tcp.append((x_w, y_w, z_w))

    def get_current_configuration(self, timeout_sec=10.0):
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

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

    def execute_trajectory(self, trajectory, collect_trail=True, spawn_trail=True, progressive=True):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Trajectory action server is unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        print("Sending trajectory goal...")

        if collect_trail:
            self.measured_actual_tcp.clear()
            self.last_spawned_index = 0
            self.trail_chunk_count = 0
            self.spawn_future = None
            self.collecting_trail = True
        else:
            self.collecting_trail = False

        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        if not send_goal_future.done():
            self.collecting_trail = False
            raise RuntimeError("Timed out waiting for controller to accept goal")

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.collecting_trail = False
            raise RuntimeError("Controller rejected the trajectory goal")

        print("Goal accepted. Executing...")

        result_future = goal_handle.get_result_async()

        # Compute dynamic timeout based on trajectory duration
        traj_duration = 10.0
        if trajectory.points:
            last_pt = trajectory.points[-1]
            traj_duration = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9

        # Progressive execution loop: process callbacks, monitor progress, spawn live trail chunks
        start_exec = time.monotonic()
        max_wait = traj_duration + 10.0
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start_exec > max_wait:
                self.collecting_trail = False
                raise RuntimeError(f"Timed out waiting for trajectory result after {max_wait:.1f} s")

            if progressive and self.collecting_trail:
                self.check_and_spawn_trail_chunk()

        # Stop collecting immediately after execution completes
        self.collecting_trail = False

        if not result_future.done():
            raise RuntimeError(f"Timed out waiting for trajectory result after {max_wait:.1f} s")

        result = result_future.result()
        if result is None:
            raise RuntimeError("No result received from controller")

        print("Controller result code:", result.result.error_code)
        print("Controller message:", result.result.error_string)

        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(f"Trajectory execution failed (status {result.status}, code {result.result.error_code}): {result.result.error_string}")

        print("SUCCESS: Trajectory completed!")
        self.report_tracking_accuracy()

        if progressive and spawn_trail and self.measured_actual_tcp:
            self.flush_remaining_trail_chunk()

        if spawn_trail and self.measured_actual_tcp:
            if self.trail_chunk_count > 0:
                print(f"SUCCESS: Progressive RED trail complete ({self.trail_chunk_count} chunks, {len(self.measured_actual_tcp)} measured points).")
            else:
                self.spawn_trail_model()

    def check_and_spawn_trail_chunk(self):
        """Check if enough new measured TCP points have accumulated to spawn a trail chunk."""
        # Ensure previous spawn request has completed so requests do not queue up and lag
        if self.spawn_future is not None and not self.spawn_future.done():
            return

        n_pts = len(self.measured_actual_tcp)
        if n_pts - self.last_spawned_index < 2:
            return

        cum_dist = 0.0
        pts_slice = self.measured_actual_tcp[self.last_spawned_index:n_pts]
        for i in range(len(pts_slice) - 1):
            p0 = pts_slice[i]
            p1 = pts_slice[i + 1]
            cum_dist += math.sqrt(
                (p1[0] - p0[0]) ** 2 +
                (p1[1] - p0[1]) ** 2 +
                (p1[2] - p0[2]) ** 2
            )

        if cum_dist >= self.chunk_dist_threshold:
            chunk_name = f"trail_chunk_{self.trail_chunk_count}"
            self.spawn_future = self._spawn_chunk(chunk_name, pts_slice)
            self.trail_chunk_count += 1
            # Next chunk starts from the last point of this chunk (zero gap)
            self.last_spawned_index = n_pts - 1

    def flush_remaining_trail_chunk(self):
        """Spawn any remaining unspawned measured TCP points as the final chunk."""
        if self.spawn_future is not None and not self.spawn_future.done():
            rclpy.spin_until_future_complete(self, self.spawn_future, timeout_sec=2.0)

        n_pts = len(self.measured_actual_tcp)
        if n_pts - self.last_spawned_index >= 2:
            pts_slice = self.measured_actual_tcp[self.last_spawned_index:n_pts]
            chunk_name = f"trail_chunk_{self.trail_chunk_count}"
            fut = self._spawn_chunk(chunk_name, pts_slice)
            if fut is not None:
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            self.trail_chunk_count += 1
            self.last_spawned_index = n_pts - 1

    def _spawn_chunk(self, model_name, raw_pts, seg_w=0.003, seg_h=0.002):
        """Build and asynchronously spawn an SDF model for a chunk of measured TCP points."""
        if len(raw_pts) < 2:
            return None

        # Downsample points (1.5 mm resolution preserves < 0.005 mm arc fidelity while minimizing visuals)
        filtered = [raw_pts[0]]
        for pt in raw_pts[1:]:
            dx = pt[0] - filtered[-1][0]
            dy = pt[1] - filtered[-1][1]
            dz = pt[2] - filtered[-1][2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist >= 0.0015:
                filtered.append(pt)

        # Ensure the final point is included so the chunk touches the current tip position
        if filtered[-1] != raw_pts[-1]:
            filtered.append(raw_pts[-1])

        if len(filtered) < 2:
            return None

        visuals = []
        for i in range(len(filtered) - 1):
            x1, y1, z1 = filtered[i]
            x2, y2, z2 = filtered[i + 1]

            dx = x2 - x1
            dy = y2 - y1
            dz = z2 - z1
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 1e-6:
                continue

            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0
            mz = (z1 + z2) / 2.0

            seg_len = dist * 1.05  # 5% overlap ensures contiguous outline
            yaw = math.atan2(dy, dx)
            pitch = -math.atan2(dz, math.sqrt(dx * dx + dy * dy))

            visuals.append(
                f"    <visual name='tv_{i}'>\n"
                f"      <pose>{mx:.6f} {my:.6f} {mz:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>\n"
                f"      <geometry>\n"
                f"        <box><size>{seg_len:.6f} {seg_w:.6f} {seg_h:.6f}</size></box>\n"
                f"      </geometry>\n"
                f"      <material>\n"
                f"        <ambient>1 0 0 1</ambient>\n"
                f"        <diffuse>1 0 0 1</diffuse>\n"
                f"      </material>\n"
                f"    </visual>\n"
            )

        if not visuals:
            return None

        visuals_str = "".join(visuals)
        sdf = (
            f"<sdf version='1.6'>\n"
            f"  <model name='{model_name}'>\n"
            f"    <static>true</static>\n"
            f"    <link name='trail_link'>\n"
            f"{visuals_str}"
            f"    </link>\n"
            f"  </model>\n"
            f"</sdf>\n"
        )

        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = sdf
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.0

        # Asynchronous call: non-blocking so trajectory execution continues seamlessly
        return self.spawn_client.call_async(req)

    def spawn_trail_model(self, model_name="trail_actual", seg_w=0.003, seg_h=0.002):
        """Construct and spawn an SDF model representing the measured TCP trail."""
        raw_pts = self.measured_actual_tcp
        if not raw_pts:
            print("No trail points were collected.")
            return

        actual_z_values = [p[2] for p in raw_pts]
        print(f"Recorded {len(raw_pts)} actual TCP feedback samples from /joint_states.")
        print(f"Actual measured TCP Z height range: {min(actual_z_values)*1000.0:.2f} .. {max(actual_z_values)*1000.0:.2f} mm in Gazebo world.")
        print("Generating continuous RED trail using 100% actual measured TCP coordinates (no projection).")

        # Filter consecutive points that are stationary/jitter (< 0.1 mm in meters = 0.0001 m)
        filtered = [raw_pts[0]]
        for pt in raw_pts[1:]:
            dx = pt[0] - filtered[-1][0]
            dy = pt[1] - filtered[-1][1]
            dz = pt[2] - filtered[-1][2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist >= 0.0001:
                filtered.append(pt)

        if len(filtered) < 2:
            print("Not enough motion detected to render a trail.")
            return

        # Downsample if number of segments exceeds 400 to keep SDF lightweight
        max_segments = 400
        if len(filtered) - 1 > max_segments:
            step = (len(filtered) - 1) / max_segments
            indices = [int(i * step) for i in range(max_segments)] + [len(filtered) - 1]
            pts = [filtered[i] for i in indices]
        else:
            pts = filtered

        visuals = []
        for i in range(len(pts) - 1):
            p1 = pts[i]
            p2 = pts[i + 1]

            x1, y1, z1 = p1
            x2, y2, z2 = p2

            dx = x2 - x1
            dy = y2 - y1
            dz = z2 - z1
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 1e-6:
                continue

            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0
            mz = (z1 + z2) / 2.0

            seg_len = dist * 1.05  # 5% overlap ensures contiguous outline without gaps
            yaw = math.atan2(dy, dx)
            pitch = -math.atan2(dz, math.sqrt(dx * dx + dy * dy))

            visuals.append(
                f"    <visual name='tv_{i}'>\n"
                f"      <pose>{mx:.6f} {my:.6f} {mz:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>\n"
                f"      <geometry>\n"
                f"        <box><size>{seg_len:.6f} {seg_w:.6f} {seg_h:.6f}</size></box>\n"
                f"      </geometry>\n"
                f"      <material>\n"
                f"        <ambient>1 0 0 1</ambient>\n"
                f"        <diffuse>1 0 0 1</diffuse>\n"
                f"      </material>\n"
                f"    </visual>\n"
            )

        if not visuals:
            print("No valid trail visuals generated.")
            return

        visuals_str = "".join(visuals)
        sdf = (
            f"<sdf version='1.6'>\n"
            f"  <model name='{model_name}'>\n"
            f"    <static>true</static>\n"
            f"    <link name='trail_link'>\n"
            f"{visuals_str}"
            f"    </link>\n"
            f"  </model>\n"
            f"</sdf>\n"
        )

        print(f"Spawning measured TCP trail '{model_name}' ({len(visuals)} segments, color=RED, width={seg_w*1000:.1f} mm)...")
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            print("WARNING: /spawn_entity service unavailable; skipping trail spawn.")
            return

        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = sdf
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.0

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None:
            res = future.result()
            if res.success:
                print(f"SUCCESS: Spawned continuous RED end-effector trail '{model_name}'")
            else:
                print(f"WARNING: SpawnEntity failed: {res.status_message}")
        else:
            print("WARNING: SpawnEntity call timed out.")

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
    parser.add_argument("--shape", choices=("line", "square", "circle"), default="line")
    parser.add_argument("--dx", type=float, default=None,
                        help="Cartesian X displacement in mm")
    parser.add_argument("--dy", type=float, default=None,
                        help="Cartesian Y displacement in mm")
    parser.add_argument("--dz", type=float, default=None,
                        help="Cartesian Z displacement in mm")
    parser.add_argument("--size", type=float, default=20.0)
    parser.add_argument("--radius", type=float, default=60.0)
    parser.add_argument("--cx", type=float, default=300.0, help="Circle center X in mm (default: 300 mm)")
    parser.add_argument("--cy", type=float, default=0.0, help="Circle center Y in mm (default: 0 mm)")
    parser.add_argument("--pen", type=float, default=15.0, help="Tip height above ground while tracing in world mm (default: 15 mm)")
    parser.add_argument("--approach", type=float, default=60.0, help="Approach point height above ground in world mm (default: 60 mm)")
    parser.add_argument("--blend-radius", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--trail", action="store_true", default=True,
        help="Record /joint_states and spawn continuous actual trail"
    )
    parser.add_argument(
        "--no-trail", dest="trail", action="store_false",
        help="Disable trail recording and spawning"
    )
    parser.add_argument(
        "--dry", action="store_true",
        help="Dry run without waiting for live robot"
    )
    args = parser.parse_args()

    # Preserve the original default: a 20 mm Y movement.
    if args.dx is None and args.dy is None and args.dz is None:
        args.dx, args.dy, args.dz = 0.0, 20.0, 0.0
    else:
        args.dx = args.dx if args.dx is not None else 0.0
        args.dy = args.dy if args.dy is not None else 0.0
        args.dz = args.dz if args.dz is not None else 0.0

    duration = args.duration
    if duration is None:
        duration = 2.0 if args.shape == "line" else (12.0 if args.shape == "circle" else 8.0)

    if not math.isfinite(duration) or duration < 2.0:
        parser.error("Duration must be at least 2 seconds")

    if args.shape == "line":
        for axis, value in (("dx", args.dx), ("dy", args.dy), ("dz", args.dz)):
            if not math.isfinite(value) or abs(value) > 20.0:
                parser.error(f"--{axis} must be between -20 and 20 mm")
    elif args.shape == "square":
        if not math.isfinite(args.size) or not 0 < args.size <= 120:
            parser.error("--size must be greater than 0 and at most 120 mm")
        if (not math.isfinite(args.blend_radius)
                or not 0 < args.blend_radius < args.size / 2):
            parser.error("--blend-radius must be positive and less than half the square size")
        if duration < 8.0:
            parser.error("Square trajectories require at least 8 seconds")
    else:
        if not math.isfinite(args.radius) or not 0 < args.radius <= 100:
            parser.error("--radius must be greater than 0 and at most 100 mm")
        if duration < 6.0:
            parser.error("Circle trajectories require at least 6 seconds")

    rclpy.init()
    node = TrajectoryPublisher()

    try:
        if args.dry:
            try:
                q_start = node.get_current_configuration(timeout_sec=0.5)
            except Exception:
                print("Dry run: live joint state not detected, validating from home (0, 0, 0)")
                q_start = (0.0, 0.0, 0.0)
        else:
            q_start = node.get_current_configuration()

        if not check_joint_limits(q_start):
            raise RuntimeError("Current configuration violates joint limits")

        start = forward_kinematics(q_start)

        print("\nCurrent TCP:", start)
        print("Selected shape:", args.shape)
        print("Duration:", duration, "seconds")

        if args.shape == "line":
            target = (
                start[0] + args.dx,
                start[1] + args.dy,
                start[2] + args.dz,
            )
            print("Target TCP:", target)

            samples = plan_cartesian_line_smooth_with_velocity(
                start, target,
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
                node.execute_trajectory(message, spawn_trail=args.trail)
            else:
                print("\nDRY RUN: No movement command sent.")
                print("All position, velocity and acceleration checks passed.")

        elif args.shape == "square":
            # SQUARE OBJECTIVE: Five-stage floor tracing sequence

            square_x = 244.0
            square_y = -60.0

            pen_world = args.pen
            approach_world = args.approach

            # Convert Gazebo world height to CAD coordinates.
            pen_cad = pen_world - 80.0
            approach_cad = approach_world - 80.0

            square_start = (square_x, square_y, pen_cad)
            approach_tcp = (square_x, square_y, approach_cad)

            print("\\n========== 5-STAGE SQUARE TRAJECTORY PLAN ==========")
            print(f"Square start: ({square_x:.1f}, {square_y:.1f}) mm")
            print(f"Side length: {args.size:.1f} mm")
            print(f"Corner radius: {args.blend_radius:.1f} mm")
            print(f"Tracing height: {pen_world:.1f} mm world")
            print(f"Approach height: {approach_world:.1f} mm world")
            print(f"Current TCP: {start}")

            # Stage 1: Home -> Approach
            s1_samples = plan_cartesian_line_smooth_with_velocity(
                start, approach_tcp,
                duration=4.0, frequency=50.0, elbow="up"
            )

            # Stage 2: Approach -> Square start
            s2_samples = plan_cartesian_line_smooth_with_velocity(
                approach_tcp, square_start,
                duration=2.0, frequency=50.0, elbow="up"
            )

            # Stage 3: Trace rounded square
            s3_samples = plan_cartesian_square(
                square_start,
                size=args.size,
                duration=duration,
                blend_radius=args.blend_radius,
                frequency=50.0,
                elbow="up",
            )

            # Stage 4: Retract
            s4_samples = plan_cartesian_line_smooth_with_velocity(
                square_start, approach_tcp,
                duration=2.0, frequency=50.0, elbow="up"
            )

            # Stage 5: Return home
            s5_samples = plan_cartesian_line_smooth_with_velocity(
                approach_tcp, start,
                duration=4.0, frequency=50.0, elbow="up"
            )

            stages = [
                ("Stage 1: Home -> Approach", s1_samples, q_start),
                ("Stage 2: Approach -> Square",
                 s2_samples, s1_samples[-1][2]),
                ("Stage 3: Square tracing",
                 s3_samples, s2_samples[-1][2]),
                ("Stage 4: Retract",
                 s4_samples, s3_samples[-1][2]),
                ("Stage 5: Return home",
                 s5_samples, s4_samples[-1][2]),
            ]

            print("\\nValidating all five square stages:")

            for name, smp, q_s in stages:
                node.validate_samples(smp, q_s)
                print(f"  [PASS] {name}: {len(smp)} samples")

            # Build all five trajectory messages
            messages = [
                build_joint_trajectory(smp)
                for _, smp, _ in stages
            ]

            if args.execute:
                print("\\n[Stage 1/5] Moving to approach point...")
                node.execute_trajectory(
                    messages[0],
                    collect_trail=False,
                    spawn_trail=False
                )

                print("\\n[Stage 2/5] Descending to square...")
                node.execute_trajectory(
                    messages[1],
                    collect_trail=False,
                    spawn_trail=False
                )

                print("\\n[Stage 3/5] Tracing square with progressive red trail...")
                node.execute_trajectory(
                    messages[2],
                    collect_trail=args.trail,
                    spawn_trail=args.trail
                )

                print("\\n[Stage 4/5] Retracting...")
                node.execute_trajectory(
                    messages[3],
                    collect_trail=False,
                    spawn_trail=False
                )

                print("\\n[Stage 5/5] Returning home...")
                node.execute_trajectory(
                    messages[4],
                    collect_trail=False,
                    spawn_trail=False
                )

                print("\\nSUCCESS: All five square stages completed!")

            else:
                print("\\nDRY RUN: All five square stages validated.")
                print("No movement command sent.")

        else:
            # CIRCLE OBJECTIVE: Full 5-stage motion planning on the floor
            cx = args.cx
            cy = args.cy
            radius = args.radius
            pen_world = args.pen
            approach_world = args.approach

            # Convert world Z heights to CAD frame (Z_cad = Z_world - 80.0 mm)
            pen_cad = pen_world - 80.0
            approach_cad = approach_world - 80.0

            # Target waypoints in CAD frame
            circle_start_tcp = (cx + radius, cy, pen_cad)
            approach_tcp = (cx + radius, cy, approach_cad)

            print(f"\n========== 5-STAGE CIRCLE TRAJECTORY PLAN ==========")
            print(f"Circle Center : ({cx:.1f}, {cy:.1f}) mm, Radius: {radius:.1f} mm")
            print(f"Tracing Height: {pen_world:.1f} mm world ({pen_cad:.1f} mm CAD)")
            print(f"Approach Ht   : {approach_world:.1f} mm world ({approach_cad:.1f} mm CAD)")
            print(f"Current Start : ({start[0]:.1f}, {start[1]:.1f}, {start[2]:.1f}) mm CAD")

            # Stage 1: Current TCP / Home -> Approach point
            s1_samples = plan_cartesian_line_smooth_with_velocity(
                start, approach_tcp, duration=4.0, frequency=50.0, elbow="up"
            )
            # Stage 2: Approach -> Circle start on floor (Descent)
            s2_samples = plan_cartesian_line_smooth_with_velocity(
                approach_tcp, circle_start_tcp, duration=2.0, frequency=50.0, elbow="up"
            )
            # Stage 3: Full 360-degree circle lap on floor
            segments = [_arc_segment((cx, cy), radius, 0.0, 2.0 * math.pi, pen_cad)]
            s3_samples = _plan_path(segments, duration=duration, frequency=50.0, elbow="up")
            # Stage 4: Circle start -> Retract to approach point
            s4_samples = plan_cartesian_line_smooth_with_velocity(
                circle_start_tcp, approach_tcp, duration=2.0, frequency=50.0, elbow="up"
            )
            # Stage 5: Approach -> Return to Start / Home
            s5_samples = plan_cartesian_line_smooth_with_velocity(
                approach_tcp, start, duration=4.0, frequency=50.0, elbow="up"
            )

            # Validate all 5 stages before execution
            print("\nValidating all 5 stages against joint position, velocity and acceleration limits:")
            stages = [
                ("Stage 1: Home -> Approach", s1_samples, q_start),
                ("Stage 2: Approach -> Descent", s2_samples, s1_samples[-1][2]),
                ("Stage 3: 360-degree Circle", s3_samples, s2_samples[-1][2]),
                ("Stage 4: Retract -> Approach", s4_samples, s3_samples[-1][2]),
                ("Stage 5: Approach -> Home", s5_samples, s4_samples[-1][2]),
            ]

            for name, smp, q_s in stages:
                node.validate_samples(smp, q_s)
                print(f"  [PASS] {name}: {len(smp)} samples validated successfully.")

            # Build trajectory messages
            msg_s1 = build_joint_trajectory(s1_samples)
            msg_s2 = build_joint_trajectory(s2_samples)
            msg_s3 = build_joint_trajectory(s3_samples)
            msg_s4 = build_joint_trajectory(s4_samples)
            msg_s5 = build_joint_trajectory(s5_samples)

            if args.execute:
                print("\n[Executing Stage 1/5] Moving to Approach Point...")
                node.execute_trajectory(msg_s1, collect_trail=False, spawn_trail=False)

                print("\n[Executing Stage 2/5] Descending to Circle Start on Floor...")
                node.execute_trajectory(msg_s2, collect_trail=False, spawn_trail=False)

                print("\n[Executing Stage 3/5] Tracing Complete 360-degree Circle on Floor (Recording Trail)...")
                node.execute_trajectory(msg_s3, collect_trail=args.trail, spawn_trail=args.trail)

                print("\n[Executing Stage 4/5] Retracting to Approach Point...")
                node.execute_trajectory(msg_s4, collect_trail=False, spawn_trail=False)

                print("\n[Executing Stage 5/5] Returning to Home...")
                node.execute_trajectory(msg_s5, collect_trail=False, spawn_trail=False)

                print("\nSUCCESS: All 5 circle demonstration stages completed!")
            else:
                print("\nDRY RUN: All 5 motion stages validated successfully. No movement command sent.")

    except Exception as error:
        print("\nERROR:", error)
        sys.exit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
