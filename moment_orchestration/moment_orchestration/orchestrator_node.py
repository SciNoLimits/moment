import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class OrchestratorNode(Node):

    def __init__(self, name: str):
        super().__init__(node_name=name)

        self.get_logger().info(
            'MOMENT Orchestrator started.'
        )


def main(args=None):
    rclpy.init(args=args)

    node = OrchestratorNode(name='orchestrator_node')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()