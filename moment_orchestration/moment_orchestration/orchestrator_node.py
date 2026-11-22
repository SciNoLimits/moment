import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
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

                ('blue_bay.x', 3.72),
                ('blue_bay.y', 8.6),
                ('blue_bay.yaw', 0.0),

                ('global_frame', 'map'),
                ('robot_frame', 'base_footprint'),
                
                ('pick_z_offset', 0.089),
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

        self.identify_object()
        
        
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
                f'Navigation failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if result.status != 4:
            self.get_logger().error(
                f'Navigation did not succeed. Status: {result.status}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not result.result.success:
            self.get_logger().error(
                f'Navigation failed: {result.result.message}'
            )
            self.set_state(MissionState.ERROR)
            return

        if self.current_state == MissionState.NAVIGATE_HOME:

            self.get_logger().info(
                'Successfully reached home.'
            )

            self.detect_objects()

        elif self.current_state == MissionState.NAVIGATE_TO_BAY:

            self.get_logger().info(
                'Successfully reached destination bay.'
            )

            self.place_object()


    def identify_object(self):
        
        self.set_state(MissionState.IDENTIFY_OBJECT)

        if not self.detected_objects:
            self.get_logger().warning('No objects available for selection.')
            self.set_state(MissionState.ERROR)
            return

        # Select the first detected object
        self.detected_object = self.detected_objects[0]

        color = self.detected_object['color']

        self.get_logger().info(
            f"Selected {color} cube for pickup."
        )

        self.select_destination(color)
        
        
    def select_destination(self, color):

        self.destination = self.get_waypoint(f'{color}_bay')

        self.get_logger().info(
            f"Destination selected for {color} cube: "
            f"x={self.destination['x']}, "
            f"y={self.destination['y']}, "
            f"yaw={self.destination['yaw']}"
        )

        self.pick_object()


    def pick_object(self):
        
        self.set_state(MissionState.PICK_OBJECT)

        object_pose = PoseStamped()

        object_pose.header.frame_id = (
            self.detected_object['position']['frame_id'] # type: ignore
        )

        object_pose.header.stamp = self.get_clock().now().to_msg()

        object_pose.pose.position.x = (
            self.detected_object['position']['x'] # type: ignore
        )

        object_pose.pose.position.y = (
            self.detected_object['position']['y'] # type: ignore
        )

        object_pose.pose.position.z = (
            self.detected_object['position']['z'] # type: ignore
            + self.get_parameter('pick_z_offset').value
        )

        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending pick goal for {self.detected_object['color']} cube. at " # type: ignore
            f"x={object_pose.pose.position.x}, "
            f"y={object_pose.pose.position.y}, "
            f"z={object_pose.pose.position.z}" 
        )

        success = self.manipulation.pick(
            object_pose,
            self.pick_goal_callback
        )

        if not success:
            self.get_logger().error('Failed to send pick goal.')
            self.set_state(MissionState.ERROR)
            
    
    def pick_goal_callback(self, future):

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Pick goal request failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Pick goal was rejected by the action server.'
            )
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().info(
            'Pick goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.pick_result_callback
        )


    def pick_result_callback(self, future):

        try:
            result = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Pick action failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if result.status != 4:
            self.get_logger().error(
                f'Pick action did not succeed. Status: {result.status}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not result.result.success:
            self.get_logger().error(
                f"Pick failed: {result.result.message}"
            )
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().info(
            f"Pick successful: {result.result.message}"
        )
        
        self.navigate_to_bay()
        
    
    def navigate_to_bay(self):

        self.set_state(MissionState.NAVIGATE_TO_BAY)

        success = self.navigation.navigate_to(
            self.destination['x'],
            self.destination['y'],
            self.destination['yaw'],
            self.navigation_goal_callback
        )

        if not success:
            self.get_logger().error(
                'Failed to send navigation goal to the destination bay.'
            )
            self.set_state(MissionState.ERROR)
            
            
    def place_object(self):

        self.set_state(MissionState.PLACE_OBJECT)

        self.get_logger().info(
            f"Placing {self.detected_object['color']} cube at destination bay." # type: ignore
        )

        success = self.manipulation.place(
            self.place_goal_callback
        )

        if not success:
            self.get_logger().error(
                'Failed to send place goal.'
            )
            self.set_state(MissionState.ERROR)
    
    
    def place_goal_callback(self, future):

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Place goal request failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Place goal was rejected by the action server.'
            )
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().info(
            'Place goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.place_result_callback
        )
       
        
    def place_result_callback(self, future):

        try:
            result = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Place action failed: {error}'
            )
            self.set_state(MissionState.ERROR)
            return

        if result.status != 4:
            self.get_logger().error(
                f'Place action did not succeed. Status: {result.status}'
            )
            self.set_state(MissionState.ERROR)
            return

        if not result.result.success:
            self.get_logger().error(
                f"Place failed: {result.result.message}"
            )
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().info(
            f"Place successful: {result.result.message}"
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