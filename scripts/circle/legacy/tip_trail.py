import math, argparse
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from gazebo_msgs.srv import SpawnEntity
from tf2_ros import Buffer, TransformListener

ap = argparse.ArgumentParser()
ap.add_argument("--pen-z", type=float, default=0.015)
ap.add_argument("--tol", type=float, default=0.010)
ap.add_argument("--gap", type=float, default=0.005)
ap.add_argument("--max", type=int, default=300)
args = ap.parse_args()

SDF = """<sdf version='1.6'><model name='%s'><static>true</static><link name='l'>
<visual name='v'><geometry><sphere><radius>0.003</radius></sphere></geometry>
<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>"""

class Trail(Node):
    def __init__(self):
        super().__init__("tip_trail")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.pts = []
        self.create_timer(0.1, self.tick)

    def tick(self):
        try:
            t = self.buf.lookup_transform("world", "end_effector", Time()).transform.translation
        except Exception:
            return
        if abs(t.z - args.pen_z) > args.tol:
            return
        if len(self.pts) >= args.max:
            return
        for q in self.pts:
            if math.hypot(t.x - q[0], t.y - q[1]) < args.gap:
                return
        self.pts.append((t.x, t.y))
        req = SpawnEntity.Request()
        req.name = "trail_%d" % len(self.pts)
        req.xml = SDF % req.name
        req.initial_pose.position.x = t.x
        req.initial_pose.position.y = t.y
        req.initial_pose.position.z = 0.005
        self.cli.call_async(req)
        if len(self.pts) % 20 == 0:
            self.get_logger().info("%d red dots placed" % len(self.pts))

rclpy.init()
rclpy.spin(Trail())
