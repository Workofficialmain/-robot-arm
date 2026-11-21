"""Reference circle guide for Gazebo Classic.

Spawns a single SDF model containing one link with many thin box visuals
arranged in a ring. This produces a continuous circular outline using
exactly one /spawn_entity call, with no visible gaps between segments.

Arguments:
  --cx     : X centre of the circle in mm (default 378.0, the robot home X)
  --cy     : Y centre of the circle in mm (default 0.0)
  --cz     : Z height of the ring in mm (default 15.0 — matching TCP tracing plane)
  --radius : Circle radius in mm (default 10.0)
  --n      : Number of arc segments (default 360)
  --name   : Model name in Gazebo (default ref_circle)
"""

import argparse
import math

import rclpy
from gazebo_msgs.srv import SpawnEntity


def build_circle_sdf(model_name, cx_m, cy_m, cz_m, radius_m, n_segments,
                     seg_w=0.008, seg_h=0.001):
    """Build an SDF string for a continuous ring outline.

    Uses N thin box visuals, each rotated and positioned on the circle
    circumference. All boxes share one link, so only one spawn call is
    required and there are no visible gaps (each box length slightly
    exceeds the arc length between adjacent centres).

    Args:
        model_name : Gazebo model name.
        cx_m, cy_m, cz_m : Centre position in metres.
        radius_m   : Circle radius in metres.
        n_segments : Number of box segments (360 gives 1-degree resolution).
        seg_w      : Width of each box segment perpendicular to the arc (m, ~8 mm).
        seg_h      : Height of each box segment (extrusion in Z, m, ~1 mm).
    """
    arc_length = 2.0 * math.pi * radius_m / n_segments
    # Make each segment slightly longer than the arc to close gaps.
    seg_len = arc_length * 1.05

    visuals = []
    for i in range(n_segments):
        angle = 2.0 * math.pi * i / n_segments
        # Centre of this segment on the ring.
        sx = cx_m + radius_m * math.cos(angle)
        sy = cy_m + radius_m * math.sin(angle)
        sz = cz_m
        # Box rotated tangentially.
        yaw = angle + math.pi / 2.0

        visuals.append(
            f"    <visual name='v{i}'>\n"
            f"      <pose>{sx:.6f} {sy:.6f} {sz:.6f} 0 0 {yaw:.6f}</pose>\n"
            f"      <geometry>\n"
            f"        <box><size>{seg_len:.6f} {seg_w:.6f} {seg_h:.6f}</size></box>\n"
            f"      </geometry>\n"
            f"      <material>\n"
            f"        <ambient>0 0 0 1</ambient>\n"
            f"        <diffuse>0 0 0 1</diffuse>\n"
            f"      </material>\n"
            f"    </visual>\n"
        )

    visuals_str = "".join(visuals)

    sdf = (
        f"<sdf version='1.6'>\n"
        f"  <model name='{model_name}'>\n"
        f"    <static>true</static>\n"
        f"    <link name='link'>\n"
        f"{visuals_str}"
        f"    </link>\n"
        f"  </model>\n"
        f"</sdf>\n"
    )
    return sdf


def main():
    ap = argparse.ArgumentParser(
        description="Spawn a continuous circle reference outline in Gazebo Classic."
    )
    ap.add_argument("--cx", type=float, default=300.0,
                    help="Circle centre X in mm (default: 300 mm)")
    ap.add_argument("--cy", type=float, default=0.0,
                    help="Circle centre Y in mm (default: 0 mm)")
    ap.add_argument("--cz", type=float, default=15.0,
                    help="Ring height in mm above ground (default: 15.0 mm)")
    ap.add_argument("--radius", type=float, default=60.0,
                    help="Circle radius in mm (default: 60 mm)")
    ap.add_argument("--n", type=int, default=72,
                    help="Number of arc segments (default: 72, 5-deg resolution)")
    ap.add_argument("--name", default="ref_circle",
                    help="Gazebo model name (default: ref_circle)")
    args = ap.parse_args()

    if args.radius <= 0:
        raise SystemExit("--radius must be positive")
    if args.n < 8:
        raise SystemExit("--n must be at least 8")

    cx_m = args.cx / 1000.0
    cy_m = args.cy / 1000.0
    cz_m = args.cz / 1000.0
    radius_m = args.radius / 1000.0

    sdf = build_circle_sdf(
        args.name, cx_m, cy_m, cz_m, radius_m, args.n
    )

    rclpy.init()
    node = rclpy.create_node("circle_guide")
    cli = node.create_client(SpawnEntity, "/spawn_entity")

    if not cli.wait_for_service(timeout_sec=10.0):
        raise SystemExit(
            "/spawn_entity not available. Is Gazebo running through ROS?"
        )

    req = SpawnEntity.Request()
    req.name = args.name
    req.xml = sdf
    # Model pose is encoded inside each visual's <pose> tag.
    req.initial_pose.position.x = 0.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 0.0

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)

    result = fut.result()
    if result is None or not result.success:
        msg = result.status_message if result else "timeout"
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"Failed to spawn circle reference: {msg}")

    print(
        f"Continuous reference circle spawned: '{args.name}' "
        f"(radius={args.radius:.1f} mm, centre=({args.cx:.1f}, {args.cy:.1f}) mm, "
        f"{args.n} segments)"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
