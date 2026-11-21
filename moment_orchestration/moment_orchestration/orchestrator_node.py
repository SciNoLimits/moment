import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from enum import Enum, auto
from moment_orchestration.perception_client import PerceptionClient
from moment_orchestration.navigation_client import NavigationClient
from moment_orchestration.manipulation_client import ManipulationClient


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
        
        # Declaring mission parameters
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

        # Mission state
        self.current_state = MissionState.IDLE

        # Currently detected object
        self.detected_object = None

        # Mission status
        self.mission_active = False
        
        # Detected objects
        self.detected_objects = []
        
        # Initializing clients
        self.perception = PerceptionClient(self)
        self.navigation = NavigationClient(self)
        self.manipulation = ManipulationClient(self)
        
        self.get_logger().info('Perception client initialized.')
        self.get_logger().info('Navigation client initialized.')
        self.get_logger().info('Manipulation client initialized.')

        self.get_logger().info('MOMENT Orchestrator initialized.')
        
        self.start_timer = self.create_timer(
            2.0,
            self.start_mission
        )
        
        
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
        
        
    def start_mission(self):

        # Execute only once
        self.start_timer.cancel()

        self.get_logger().info('Starting MOMENT mission...')

        self.mission_active = True

        self.navigate_home()
        
        
    def navigate_home(self):

        self.set_state(MissionState.NAVIGATE_HOME)

        home = self.get_waypoint('home')

        success = self.navigation.navigate_to(
            home['x'],
            home['y'],
            home['yaw'],
            self.navigation_goal_callback
        )

        if not success:
            self.set_state(MissionState.ERROR)
            
            
    def navigation_goal_callback(self, future):

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Navigation goal request failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Navigation goal rejected.'
            )
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().info(
            'Navigation goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )
        
        
    def navigation_result_callback(self, future):

        try:
            result = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Navigation execution failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if result.status != 4:
            self.get_logger().error(
                f'Navigation failed. Status: {result.status}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not result.result.success:
            self.get_logger().error(
                'Navigation action reported failure.'
            )
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().info(
            'Successfully reached home.'
        )

        self.detect_objects()


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