import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Mover(Node):
    def __init__(self, pos):
        super().__init__("move_joints")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.pos = pos
        self.create_timer(0.05, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_base_rotate", "joint_shoulder", "joint_elbow"]
        msg.position = self.pos
        self.pub.publish(msg)

args = [float(x) for x in sys.argv[1:4]] if len(sys.argv) >= 4 else [0.0, 0.0, 0.0]
rclpy.init()
rclpy.spin(Mover(args))
