import math, argparse, time
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_msgs.srv import SpawnEntity
from tf2_ros import Buffer, TransformListener

SHOULDER = (0.0, 0.0, 52.0)
ELBOW = (30.0, 0.0, 288.8)
TIP = (378.0, 0.0, 293.0)
L1v = (ELBOW[0] - SHOULDER[0], ELBOW[2] - SHOULDER[2])
L2v = (TIP[0] - ELBOW[0], TIP[2] - ELBOW[2])
L1 = math.hypot(*L1v)
L2 = math.hypot(*L2v)
PHI1_0 = math.atan2(L1v[1], L1v[0])
PHI2_0 = math.atan2(L2v[1], L2v[0])
LIMITS = (3.0, 1.5, 1.5)
LIFT = 80.0
JOINTS = ["joint_base_rotate", "joint_shoulder", "joint_elbow"]

def ik(x, y, z):
    r = math.hypot(x, y)
    q1 = math.atan2(y, x)
    zt = z - SHOULDER[2]
    c = (r * r + zt * zt - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1.0:
        raise SystemExit("point out of reach: (%.0f, %.0f, %.0f)" % (x, y, z))
    delta = -math.acos(c)
    phi1 = math.atan2(zt, r) - math.atan2(L2 * math.sin(delta), L1 + L2 * math.cos(delta))
    phi2 = phi1 + delta
    q2 = PHI1_0 - phi1
    q3 = (PHI2_0 - phi2) - q2
    return (q1, q2, q3)

ap = argparse.ArgumentParser()
ap.add_argument("--cx", type=float, default=300.0)
ap.add_argument("--cy", type=float, default=0.0)
ap.add_argument("--radius", type=float, default=60.0)
ap.add_argument("--pen", type=float, default=15.0, help="tip height above ground while tracing (mm)")
ap.add_argument("--approach", type=float, default=60.0, help="approach point height above ground (mm)")
ap.add_argument("--period", type=float, default=20.0, help="seconds for the one lap")
ap.add_argument("--points", type=int, default=120)
ap.add_argument("--dry", action="store_true")
a = ap.parse_args()

sx, sy = a.cx + a.radius, a.cy
q_appr = ik(sx, sy, a.approach - LIFT)
N_DESC = 20
descent = [ik(sx, sy, (a.approach + (a.pen - a.approach) * i / N_DESC) - LIFT) for i in range(1, N_DESC + 1)]
circle = []
for i in range(1, a.points + 1):
    th = 2 * math.pi * i / a.points
    circle.append(ik(a.cx + a.radius * math.cos(th), a.cy + a.radius * math.sin(th), a.pen - LIFT))

allq = [q_appr] + descent + circle
names = ["base", "shoulder", "elbow"]
for k in range(3):
    lo = min(q[k] for q in allq)
    hi = max(q[k] for q in allq)
    print("%-8s range: %.2f .. %.2f rad (limit +-%.2f)" % (names[k], lo, hi, LIMITS[k]))
    if max(abs(lo), abs(hi)) > LIMITS[k]:
        raise SystemExit("A joint would exceed its limit. Change --cx/--radius/--pen.")
if a.dry:
    raise SystemExit("dry run only, nothing sent")

def dur(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))

def mk(q, t):
    p = JointTrajectoryPoint()
    p.positions = list(q)
    p.time_from_start = dur(t)
    return p

SDF = ("<sdf version='1.6'><model name='%s'><static>true</static><link name='l'>"
       "<visual name='v'><geometry><sphere><radius>%s</radius></sphere></geometry>"
       "<material><ambient>%s 1</ambient><diffuse>%s 1</diffuse></material></visual></link></model></sdf>")

rclpy.init()
node = rclpy.create_node("circle_demo")
buf = Buffer()
TransformListener(buf, node)
ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10.0):
    raise SystemExit("arm_controller is not available. Check: ros2 control list_controllers")
cli = node.create_client(SpawnEntity, "/spawn_entity")
if not cli.wait_for_service(timeout_sec=10.0):
    raise SystemExit("/spawn_entity not available. Is Gazebo running through ROS?")

def tip():
    for _ in range(100):
        try:
            t = buf.lookup_transform("world", "end_effector", Time()).transform.translation
            return t.x, t.y, t.z
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.05)
    raise SystemExit("no TF world -> end_effector. Is robot_state_publisher running?")

def settle(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def spawn(name, radius, rgb, x, y, z, wait=False):
    req = SpawnEntity.Request()
    req.name = name
    req.xml = SDF % (name, radius, rgb, rgb)
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    fut = cli.call_async(req)
    if wait:
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

def run(points, tick=None):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = points
    fut = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        raise SystemExit("goal rejected by arm_controller")
    res = gh.get_result_async()
    while not res.done():
        rclpy.spin_once(node, timeout_sec=0.05)
        if tick:
            tick()

print("1/6 going home ...")
run([mk((0.0, 0.0, 0.0), 6.0)])
settle(0.8)
x, y, z = tip()
spawn("mark_home", 0.008, "0 0.3 1", x, y, z, wait=True)
print("    blue dot placed at home (%.0f, %.0f, %.0f mm)" % (x * 1000, y * 1000, z * 1000))

print("2/6 moving to the approach point above the start of the ring ...")
run([mk(q_appr, 8.0)])
settle(0.8)
x, y, z = tip()
spawn("mark_approach", 0.008, "1 0 0", x, y, z, wait=True)
print("    red approach dot placed (%.0f mm above the ground)" % (z * 1000))

print("3/6 descending to the ring ...")
run([mk(q, 4.0 * (i + 1) / N_DESC) for i, q in enumerate(descent)])
settle(0.5)

print("4/6 tracing the circle once ...")
dots = []
def tick():
    x, y, z = tip()
    if dots and math.hypot(x - dots[-1][0], y - dots[-1][1]) < 0.006:
        return
    dots.append((x, y))
    spawn("trail_%d" % len(dots), 0.003, "1 0 0", x, y, 0.006)

run([mk(q, a.period * (i + 1) / a.points) for i, q in enumerate(circle)], tick)
settle(1.0)
print("    lap finished: %d red dots on the ring" % len(dots))

print("5/6 lifting back up to the approach point ...")
up = [descent[i] for i in range(N_DESC - 2, -1, -1)] + [q_appr]
run([mk(q, 4.0 * (i + 1) / len(up)) for i, q in enumerate(up)])
settle(0.8)

print("6/6 returning to home ...")
run([mk((0.0, 0.0, 0.0), 8.0)])
settle(0.5)
print("done")
node.destroy_node()
rclpy.shutdown()
