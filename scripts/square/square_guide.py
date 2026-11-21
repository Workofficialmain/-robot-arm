"""Reference rounded-square guide for Gazebo Classic.

Spawns a single SDF model containing one link with many thin box visuals
arranged as a rounded-square outline. The shape exactly matches the
advanced trajectory planner's _rounded_square_path geometry:
  - Four straight edges
  - Four circular arc corners with the configured blend radius

Using one /spawn_entity call → no visible gaps, no performance issues.

Arguments:
  --cx          : Square start corner X in mm (default 378.0)
  --cy          : Square start corner Y in mm (default 0.0)
  --cz          : Z height in mm above ground (default 1.0)
  --size        : Square side length in mm (default 20.0)
  --blend-radius: Corner blend radius in mm (default 4.0)
  --n-corner    : Number of segments per 90-degree arc corner (default 24)
  --n-edge      : Number of segments per straight edge (default 60)
  --name        : Gazebo model name (default ref_square)
"""

import argparse
import math

import rclpy
from gazebo_msgs.srv import SpawnEntity


def _box_visual(name, x, y, z, length, width, height, yaw):
    """Return an SDF <visual> element for a thin oriented box."""
    return (
        f"    <visual name='{name}'>\n"
        f"      <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>\n"
        f"      <geometry>\n"
        f"        <box><size>{length:.6f} {width:.6f} {height:.6f}</size></box>\n"
        f"      </geometry>\n"
        f"      <material>\n"
        f"        <ambient>0 0 0 1</ambient>\n"
        f"        <diffuse>0 0 0 1</diffuse>\n"
        f"      </material>\n"
        f"    </visual>\n"
    )


def build_rounded_square_sdf(model_name, x0_m, y0_m, cz_m,
                              size_m, blend_m,
                              n_corner, n_edge,
                              seg_w=0.008, seg_h=0.001):
    """Build SDF for a rounded-square outline matching the trajectory planner.

    The start corner (x0_m, y0_m) corresponds to the robot's current TCP
    position when the trajectory starts.  The shape is:

        (x0, y0)  → bottom edge →  (x0 + size - 2r, y0)
        bottom-right arc (centre at x0+size-2r, y0+r), −π/2 → 0
        → right edge up → top-right arc → top edge left → top-left arc
        → left edge down → bottom-left arc back to (x0, y0)

    This exactly replicates the _rounded_square_path in shape_planner.py.
    """
    r = blend_m
    s = size_m
    visuals = []
    idx = 0

    def add_line(ax, ay, bx, by, n):
        nonlocal idx
        length = math.hypot(bx - ax, by - ay)
        if length < 1e-9:
            return
        yaw = math.atan2(by - ay, bx - ax)
        seg_len = length / n * 1.05
        for k in range(n):
            t = (k + 0.5) / n
            px = ax + t * (bx - ax)
            py = ay + t * (by - ay)
            visuals.append(_box_visual(f"v{idx}", px, py, cz_m,
                                       seg_len, seg_w, seg_h, yaw))
            idx += 1

    def add_arc(cx, cy, rad, a_start, a_end, n):
        nonlocal idx
        arc_len = rad * abs(a_end - a_start)
        seg_len = arc_len / n * 1.05
        for k in range(n):
            t = (k + 0.5) / n
            angle = a_start + t * (a_end - a_start)
            px = cx + rad * math.cos(angle)
            py = cy + rad * math.sin(angle)
            # Tangent direction
            tangent_angle = angle + math.pi / 2.0
            visuals.append(_box_visual(f"v{idx}", px, py, cz_m,
                                       seg_len, seg_w, seg_h, tangent_angle))
            idx += 1

    # Bottom edge
    add_line(x0_m, y0_m, x0_m + s - 2*r, y0_m, n_edge)
    # Bottom-right arc: centre (x0 + s - 2r, y0 + r), from -π/2 to 0
    add_arc(x0_m + s - 2*r, y0_m + r, r, -math.pi/2, 0, n_corner)
    # Right edge
    add_line(x0_m + s - r, y0_m + r, x0_m + s - r, y0_m + s - r, n_edge)
    # Top-right arc: centre (x0 + s - 2r, y0 + s - r), from 0 to π/2
    add_arc(x0_m + s - 2*r, y0_m + s - r, r, 0, math.pi/2, n_corner)
    # Top edge
    add_line(x0_m + s - 2*r, y0_m + s, x0_m, y0_m + s, n_edge)
    # Top-left arc: centre (x0, y0 + s - r), from π/2 to π
    add_arc(x0_m, y0_m + s - r, r, math.pi/2, math.pi, n_corner)
    # Left edge
    add_line(x0_m - r, y0_m + s - r, x0_m - r, y0_m + r, n_edge)
    # Bottom-left arc: centre (x0, y0 + r), from π to 3π/2
    add_arc(x0_m, y0_m + r, r, math.pi, 3*math.pi/2, n_corner)

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
        description=(
            "Spawn a continuous rounded-square reference outline in Gazebo Classic. "
            "The geometry exactly matches the advanced trajectory planner."
        )
    )
    ap.add_argument("--cx", type=float, default=378.0,
                    help="Start corner X in mm (should match robot TCP X)")
    ap.add_argument("--cy", type=float, default=0.0,
                    help="Start corner Y in mm (should match robot TCP Y)")
    ap.add_argument("--cz", type=float, default=1.0,
                    help="Height above ground in mm (default: 1 mm)")
    ap.add_argument("--size", type=float, default=20.0,
                    help="Square side length in mm (default: 20 mm)")
    ap.add_argument("--blend-radius", type=float, default=4.0,
                    help="Corner blend radius in mm (default: 4 mm)")
    ap.add_argument("--n-corner", type=int, default=8,
                    help="Segments per 90-degree corner arc (default: 8)")
    ap.add_argument("--n-edge", type=int, default=4,
                    help="Segments per straight edge (default: 4)")
    ap.add_argument("--name", default="ref_square",
                    help="Gazebo model name (default: ref_square)")
    args = ap.parse_args()

    if args.size <= 0:
        raise SystemExit("--size must be positive")
    if args.blend_radius <= 0 or args.blend_radius >= args.size / 2:
        raise SystemExit(
            "--blend-radius must be > 0 and < size/2 "
            f"(got {args.blend_radius} mm, size={args.size} mm)"
        )

    cx_m = args.cx / 1000.0
    cy_m = args.cy / 1000.0
    cz_m = args.cz / 1000.0
    size_m = args.size / 1000.0
    blend_m = args.blend_radius / 1000.0

    sdf = build_rounded_square_sdf(
        args.name,
        cx_m, cy_m, cz_m,
        size_m, blend_m,
        args.n_corner, args.n_edge,
    )

    rclpy.init()
    node = rclpy.create_node("square_guide")
    cli = node.create_client(SpawnEntity, "/spawn_entity")

    if not cli.wait_for_service(timeout_sec=10.0):
        raise SystemExit(
            "/spawn_entity not available. Is Gazebo running through ROS?"
        )

    req = SpawnEntity.Request()
    req.name = args.name
    req.xml = sdf
    req.initial_pose.position.x = 0.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 0.0

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)

    result = fut.result()
    if result is None or not result.success:
        msg = result.status_message if result else "timeout"
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"Failed to spawn square reference: {msg}")

    n_total = 4 * args.n_edge + 4 * args.n_corner
    print(
        f"Continuous reference rounded-square spawned: '{args.name}' "
        f"(size={args.size:.1f} mm, blend_r={args.blend_radius:.1f} mm, "
        f"start=({args.cx:.1f}, {args.cy:.1f}) mm, {n_total} segments)"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
