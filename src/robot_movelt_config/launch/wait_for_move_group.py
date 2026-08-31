import sys

import rclpy
from moveit_msgs.srv import GetPlanningScene


def main():
    rclpy.init()
    node = rclpy.create_node("wait_for_move_group")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    node.get_logger().info("Waiting for MoveGroup planning scene service before starting RViz...")
    while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
