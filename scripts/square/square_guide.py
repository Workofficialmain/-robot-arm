import math, argparse
import rclpy
from gazebo_msgs.srv import SpawnEntity

ap = argparse.ArgumentParser()
ap.add_argument("--cx", type=float, default=300.0)
ap.add_argument("--cy", type=float, default=0.0)
ap.add_argument("--side", type=float, default=100.0)
ap.add_argument("--step", type=float, default=4.0, help="spacing between black discs (mm)")
a = ap.parse_args()

SDF = """<sdf version='1.6'><model name='%s'><static>true</static><link name='l'>
<visual name='v'><geometry><cylinder><radius>0.007</radius><length>0.002</length></cylinder></geometry>
<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual></link></model></sdf>"""

h = a.side / 2.0
corners = [(a.cx + h, a.cy - h), (a.cx + h, a.cy + h), (a.cx - h, a.cy + h), (a.cx - h, a.cy - h)]
n_edge = max(1, int(round(a.side / a.step)))
pts = []
for e in range(4):
    x0, y0 = corners[e]
    x1, y1 = corners[(e + 1) % 4]
    for k in range(n_edge):
        f = k / n_edge
        pts.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))

rclpy.init()
node = rclpy.create_node("square_guide")
cli = node.create_client(SpawnEntity, "/spawn_entity")
if not cli.wait_for_service(timeout_sec=10.0):
    raise SystemExit("/spawn_entity not available. Is Gazebo running through ROS?")

for i, (x, y) in enumerate(pts):
    req = SpawnEntity.Request()
    req.name = "guide_%d" % i
    req.xml = SDF % req.name
    req.initial_pose.position.x = x / 1000.0
    req.initial_pose.position.y = y / 1000.0
    req.initial_pose.position.z = 0.0015
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
print("black square drawn on the ground: %d discs" % len(pts))
node.destroy_node()
rclpy.shutdown()
