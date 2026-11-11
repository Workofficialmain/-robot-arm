import math, argparse
import rclpy
from gazebo_msgs.srv import SpawnEntity

ap = argparse.ArgumentParser()
ap.add_argument("--cx", type=float, default=300.0)
ap.add_argument("--cy", type=float, default=0.0)
ap.add_argument("--radius", type=float, default=60.0)
ap.add_argument("--n", type=int, default=120)
a = ap.parse_args()

SDF = """<sdf version='1.6'><model name='%s'><static>true</static><link name='l'>
<visual name='v'><geometry><cylinder><radius>0.007</radius><length>0.002</length></cylinder></geometry>
<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual></link></model></sdf>"""

rclpy.init()
node = rclpy.create_node("circle_guide")
cli = node.create_client(SpawnEntity, "/spawn_entity")
if not cli.wait_for_service(timeout_sec=10.0):
    raise SystemExit("/spawn_entity not available. Is Gazebo running through ROS?")

for i in range(a.n):
    th = 2 * math.pi * i / a.n
    req = SpawnEntity.Request()
    req.name = "guide_%d" % i
    req.xml = SDF % req.name
    req.initial_pose.position.x = (a.cx + a.radius * math.cos(th)) / 1000.0
    req.initial_pose.position.y = (a.cy + a.radius * math.sin(th)) / 1000.0
    req.initial_pose.position.z = 0.0015
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
print("black ring drawn on the ground: %d discs" % a.n)
node.destroy_node()
rclpy.shutdown()
