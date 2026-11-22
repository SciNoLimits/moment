from rclpy.action import ActionClient

from moment_interfaces.action import Pick, Place


class ManipulationClient:

    def __init__(self, node):
        self.node = node

        # Pick action client
        self.pick_client = ActionClient(
            self.node,
            Pick,
            '/pick'
        )

        # Place action client
        self.place_client = ActionClient(
            self.node,
            Place,
            '/place'
        )
        
        
    def pick(self, object_pose, callback):

        if not self.pick_client.server_is_ready():
            self.node.get_logger().info(
                'Waiting for pick action server...'
            )

            if not self.pick_client.wait_for_server(timeout_sec=1.0):
                self.node.get_logger().warning(
                    'Pick action server is unavailable.'
                )
                return False

        goal = Pick.Goal()
        goal.object_pose = object_pose

        self.node.get_logger().info(
            'Sending pick goal...'
        )

        future = self.pick_client.send_goal_async(goal)

        future.add_done_callback(callback)

        return True
    
    
    def place(self, callback):

        if not self.place_client.server_is_ready():
            self.node.get_logger().info(
                'Waiting for place action server...'
            )

            if not self.place_client.wait_for_server(timeout_sec=1.0):
                self.node.get_logger().warning(
                    'Place action server is unavailable.'
                )
                return False

        goal = Place.Goal()

        self.node.get_logger().info(
            'Sending place goal...'
        )

        future = self.place_client.send_goal_async(goal)

        future.add_done_callback(callback)

        return True