import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from enum import Enum, auto
from moment_orchestration.perception_client import PerceptionClient


class MissionState(Enum):
    IDLE = auto()
    NAVIGATE_HOME = auto()
    DETECT_OBJECTS = auto()
    IDENTIFY_OBJECT = auto()
    PICK_OBJECT = auto()
    NAVIGATE_TO_BAY = auto()
    PLACE_OBJECT = auto()
    RETURN_HOME = auto()
    MISSION_COMPLETE = auto()
    ERROR = auto()


class OrchestratorNode(Node):

    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        # Declare mission parameters
        self.declare_parameters(
            namespace='',
            parameters=[ # type: ignore[arg-type]
        		('home.x', 0.0),
                ('home.y', 0.0),
                ('home.yaw', 0.0),

                ('red_bay.x', -4.7),
                ('red_bay.y', 7.95),
                ('red_bay.yaw', 1.57079632679),

                ('green_bay.x', -0.35),
                ('green_bay.y', 8.10),
                ('green_bay.yaw', 1.57079632679),

                ('blue_bay.x', 3.7),
                ('blue_bay.y', 8.6),
                ('blue_bay.yaw', 0.0),

                ('global_frame', 'map'),
                ('robot_frame', 'base_footprint'),
            ]
        )
        
        self.perception = PerceptionClient(self)
        self.detected_objects = []

        # Mission state
        self.current_state = MissionState.IDLE

        # Currently detected object
        self.detected_object = None

        # Mission status
        self.mission_active = False

        self.get_logger().info('MOMENT Orchestrator initialized.')
        
        
    def get_waypoint(self, name):
        return {
            'x': self.get_parameter(f'{name}.x').value,
            'y': self.get_parameter(f'{name}.y').value,
            'yaw': self.get_parameter(f'{name}.yaw').value,
        }
        
        
    def set_state(self, new_state):
        self.current_state = new_state

        self.get_logger().info(
            f'Mission state changed to: {self.current_state.name}'
        )
        
        
    def detect_objects(self):

        self.set_state(MissionState.DETECT_OBJECTS)

        success = self.perception.detect_objects(
            self.detection_callback
        )

        if not success:
            self.set_state(MissionState.ERROR)
            
    
    def detection_callback(self, future):

        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Perception service failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        self.detected_objects = self.perception.parse_response(response)

        if not self.detected_objects:
            self.get_logger().info(
                'No cubes detected. Mission complete.'
            )
            self.set_state(MissionState.MISSION_COMPLETE)
            return

        self.get_logger().info(
            f'Detected {len(self.detected_objects)} cube(s).'
        )

        for obj in self.detected_objects:
            self.get_logger().info(
                f"{obj['color']} cube detected at "
                f"x={obj['position']['x']:.4f}, "
                f"y={obj['position']['y']:.4f}, "
                f"z={obj['position']['z']:.4f}"
            )

        self.set_state(MissionState.IDENTIFY_OBJECT)


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