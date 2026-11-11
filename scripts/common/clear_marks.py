import re, subprocess
import rclpy
from gazebo_msgs.srv import DeleteEntity

out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True).stdout
names = sorted(set(re.findall(r"\b(?:guide_\d+|trail_\d+|mark_home|mark_approach)\b", out)))
rclpy.init()
node = rclpy.create_node("clear_marks")
cli = node.create_client(DeleteEntity, "/delete_entity")
cli.wait_for_service(timeout_sec=10.0)
for n in names:
    req = DeleteEntity.Request()
    req.name = n
    rclpy.spin_until_future_complete(node, cli.call_async(req), timeout_sec=3.0)
print("removed %d markers" % len(names))
