import re
import rclpy
from gazebo_msgs.srv import GetModelList, DeleteEntity

rclpy.init()
node = rclpy.create_node("clear_marks")

get_cli = node.create_client(GetModelList, "/get_model_list")
del_cli = node.create_client(DeleteEntity, "/delete_entity")

if not get_cli.wait_for_service(timeout_sec=5.0) or not del_cli.wait_for_service(timeout_sec=5.0):
    print("WARNING: Gazebo services not available for clearing marks.")
    node.destroy_node()
    rclpy.shutdown()
    exit(0)

fut = get_cli.call_async(GetModelList.Request())
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
resp = fut.result()

deleted = 0
if resp is not None and resp.success:
    pattern = re.compile(r"^(?:guide_\d+|trail_\d+|trail_chunk_\d+|mark_home|mark_approach|ref_circle|ref_square|trail_actual|test_speed_\d+|bench_.*)$")
    targets = [m for m in resp.model_names if pattern.match(m)]
    for name in targets:
        req = DeleteEntity.Request()
        req.name = name
        del_fut = del_cli.call_async(req)
        rclpy.spin_until_future_complete(node, del_fut, timeout_sec=3.0)
        deleted += 1

print(f"removed {deleted} markers")
node.destroy_node()
rclpy.shutdown()
