import math

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moment_interfaces.action import GoToGoal

class NavigationClient:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, GoToGoal, 'go_to_goal')

    def navigate_to(self, x, y, yaw, callback):

        if not self.client.server_is_ready():
            self.node.get_logger().info(
                'Waiting for navigation action server...'
            )

            if not self.client.wait_for_server(timeout_sec=3.0):
                self.node.get_logger().warning(
                    'Navigation action server is unavailable.'
                )
                return False

        goal = GoToGoal.Goal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = self.node.get_clock().now().to_msg()

        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.node.get_logger().info(
            f'Navigating to x={x}, y={y}, yaw={yaw}'
        )

        future = self.client.send_goal_async(goal)

        future.add_done_callback(callback)

        return True