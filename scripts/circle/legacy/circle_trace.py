import math, argparse, time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Measured pivots (mm, CAD frame, base origin at 0,0,0)
SHOULDER = (0.0, 0.0, 52.0)
ELBOW = (30.0, 0.0, 288.8)
TIP = (378.0, 0.0, 293.0)

L1v = (ELBOW[0] - SHOULDER[0], ELBOW[2] - SHOULDER[2])
L2v = (TIP[0] - ELBOW[0], TIP[2] - ELBOW[2])
L1 = math.hypot(*L1v)
L2 = math.hypot(*L2v)
PHI1_0 = math.atan2(L1v[1], L1v[0])
PHI2_0 = math.atan2(L2v[1], L2v[0])
LIM = {"base": 3.0, "shoulder": 1.5, "elbow": 1.5}

def ik(x, y, z):
    r = math.hypot(x, y)
    q1 = math.atan2(y, x)
    zt = z - SHOULDER[2]
    c = (r * r + zt * zt - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1.0:
        raise ValueError("point out of reach: (%.0f, %.0f, %.0f)" % (x, y, z))
    delta = -math.acos(c)  # elbow-up
    phi1 = math.atan2(zt, r) - math.atan2(L2 * math.sin(delta), L1 + L2 * math.cos(delta))
    phi2 = phi1 + delta
    q2 = PHI1_0 - phi1
    q3 = (PHI2_0 - phi2) - q2
    return q1, q2, q3

def fk(q1, q2, q3):
    phi1 = PHI1_0 - q2
    phi2 = PHI2_0 - q2 - q3
    r = L1 * math.cos(phi1) + L2 * math.cos(phi2)
    z = SHOULDER[2] + L1 * math.sin(phi1) + L2 * math.sin(phi2)
    return r * math.cos(q1), r * math.sin(q1), z

def dur(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))

ap = argparse.ArgumentParser()
ap.add_argument("--cx", type=float, default=300.0)
ap.add_argument("--cy", type=float, default=0.0)
ap.add_argument("--cz", type=float, default=-65.0)
ap.add_argument("--radius", type=float, default=60.0)
ap.add_argument("--period", type=float, default=20.0)
ap.add_argument("--loops", type=int, default=2)
ap.add_argument("--points", type=int, default=120)
ap.add_argument("--dry", action="store_true")
a = ap.parse_args()

pts = []
for i in range(a.points + 1):
    th = 2 * math.pi * i / a.points
    pts.append(ik(a.cx + a.radius * math.cos(th), a.cy + a.radius * math.sin(th), a.cz))

lo = [min(p[k] for p in pts) for k in range(3)]
hi = [max(p[k] for p in pts) for k in range(3)]
err = 0.0
for i, p in enumerate(pts):
    th = 2 * math.pi * i / a.points
    t = (a.cx + a.radius * math.cos(th), a.cy + a.radius * math.sin(th), a.cz)
    f = fk(*p)
    err = max(err, math.dist(t, f))
print("L1 = %.1f mm, L2 = %.1f mm" % (L1, L2))
print("base     range: %.2f .. %.2f rad" % (lo[0], hi[0]))
print("shoulder range: %.2f .. %.2f rad" % (lo[1], hi[1]))
print("elbow    range: %.2f .. %.2f rad" % (lo[2], hi[2]))
print("max IK/FK check error: %.4f mm" % err)

if max(abs(lo[0]), abs(hi[0])) > LIM["base"] or max(abs(lo[1]), abs(hi[1])) > LIM["shoulder"] or max(abs(lo[2]), abs(hi[2])) > LIM["elbow"]:
    raise SystemExit("A joint would exceed its limit. Change --cx/--cz/--radius.")
if a.dry:
    raise SystemExit("dry run only, nothing sent")

class Pub(Node):
    def __init__(self):
        super().__init__("circle_trace")
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

rclpy.init()
node = Pub()
t0 = time.time()
while node.pub.get_subscription_count() == 0 and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.pub.get_subscription_count() == 0:
    raise SystemExit("arm_controller is not listening. Load the controllers first.")

msg = JointTrajectory()
msg.joint_names = ["joint_base_rotate", "joint_shoulder", "joint_elbow"]
approach = 8.0
step = a.period / a.points
t = approach
p = JointTrajectoryPoint(); p.positions = list(pts[0]); p.time_from_start = dur(t)
msg.points.append(p)
for loop in range(a.loops):
    for i in range(1, a.points + 1):
        t += step
        p = JointTrajectoryPoint(); p.positions = list(pts[i]); p.time_from_start = dur(t)
        msg.points.append(p)
node.pub.publish(msg)
print("trajectory sent: %.0f s total (%.0f s approach + %d loops of %.0f s)" % (t, approach, a.loops, a.period))
rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
