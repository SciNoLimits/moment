import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from enum import Enum, auto
from moment_orchestration.perception_client import PerceptionClient
from moment_orchestration.navigation_client import NavigationClient
from moment_orchestration.manipulation_client import ManipulationClient
from tf2_ros import Buffer, TransformListener, TransformException # type: ignore
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.clock import ClockType


class MissionState(Enum):
    IDLE = auto()
    NAVIGATE_HOME = auto()
    DETECT_OBJECTS = auto()
    IDENTIFY_OBJECT = auto()
    APPROACH_OBJECT = auto()
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

                ('blue_bay.x', 3.735),
                ('blue_bay.y', 8.6),
                ('blue_bay.yaw', 0.0),

                ('global_frame', 'map'),
                ('robot_frame', 'base_footprint'),
                
                ('pick_z_offset', 0.089),
                
                ('preferred_pick_x', 0.200),
                ('approach_tolerance', 0.08),
                
                ('object_detector_delay', 1.0),
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
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )
        
        self.target_color = None
        self.approach_pending = False
        self.approach_pending = False
        self.detection_timer = None
        
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
        
        # Handle refreshed perception after approaching an object
        if self.approach_pending:

            matching_objects = [
                obj for obj in self.detected_objects
                if obj['color'] == self.target_color
            ]

            if not matching_objects:
                self.get_logger().error(
                    f'Could not detect the target {self.target_color} cube '
                    'after approaching.'
                )
                self.approach_pending = False
                self.set_state(MissionState.ERROR)
                return

            # Update the target with its latest position
            self.detected_object = matching_objects[0]

            self.approach_pending = False

            self.get_logger().info(
                f'Redetected {self.target_color} cube: '
                f'{self.detected_object["position"]}'
            )

            self.pick_object()
            return

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
        
        object_detector_delay = self.get_parameter('object_detector_delay').value

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
            self.schedule_detection()
            
        elif self.current_state == MissionState.APPROACH_OBJECT:
            self.get_logger().info(
                'Approach completed. Detecting objects again...'
            )
            self.schedule_detection()

        elif self.current_state == MissionState.NAVIGATE_TO_BAY:

            self.get_logger().info(
                'Successfully reached destination bay.'
            )
            self.place_object()
            
        elif self.current_state == MissionState.RETURN_HOME:

            self.get_logger().info(
                'Successfully returned home.'
            )
            self.schedule_detection()


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

        self.target_color = color

        self.approach_or_pick()


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
                f'Pick action did not succeed. '
                f'Status: {result.status}. '
                f'Message: {result.result.message}'
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
        
        self.return_home()


    def return_home(self):

        self.set_state(MissionState.RETURN_HOME)

        home = self.get_waypoint('home')

        success = self.navigation.navigate_to(
            home['x'],
            home['y'],
            home['yaw'],
            self.navigation_goal_callback
        )

        if not success:
            self.get_logger().error(
                'Failed to send navigation goal to home.'
            )
            self.set_state(MissionState.ERROR)


    def calculate_approach_goal(self, forward_distance):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('global_frame').value, # type: ignore
                self.get_parameter('robot_frame').value, # type: ignore
                Time(
                    nanoseconds=0,
                    clock_type=ClockType.ROS_TIME
                ),
                timeout=Duration(seconds=3.0)
            )

        except TransformException as error:
            self.get_logger().error(
                f'Could not get robot pose: {error}'
            )
            return None

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        orientation = transform.transform.rotation

        # Extract robot yaw from quaternion
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            orientation.y ** 2
            + orientation.z ** 2
        )

        robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Calculate forward displacement in map frame
        goal_x = robot_x + forward_distance * math.cos(robot_yaw)
        goal_y = robot_y + forward_distance * math.sin(robot_yaw)

        return {
            'x': goal_x,
            'y': goal_y,
            'yaw': robot_yaw
        }
        
        
    def approach_or_pick(self):

        object_x = self.detected_object['position']['x'] # type: ignore

        preferred_pick_x = self.get_parameter(
            'preferred_pick_x'
        ).value

        tolerance = self.get_parameter(
            'approach_tolerance'
        ).value

        forward_distance = object_x - preferred_pick_x

        if forward_distance > tolerance:
            self.get_logger().info(
                f'Object is {object_x:.3f} m ahead. '
                f'Robot needs to approach by {forward_distance:.3f} m.'
            )

            self.approach_object(forward_distance)

        else:
            self.get_logger().info(
                'Object is within the preferred picking distance.'
            )
            self.pick_object()
            
            
    def approach_object(self, forward_distance):

        approach_goal = self.calculate_approach_goal(
            forward_distance
        )

        if approach_goal is None:
            self.set_state(MissionState.ERROR)
            return

        self.approach_pending = True

        self.set_state(MissionState.APPROACH_OBJECT)

        self.get_logger().info(
            f'Approaching object: '
            f'x={approach_goal["x"]:.3f}, '
            f'y={approach_goal["y"]:.3f}'
        )

        goal_sent = self.navigation.navigate_to(
            approach_goal['x'],
            approach_goal['y'],
            approach_goal['yaw'],
            self.navigation_goal_callback
        )

        if not goal_sent:
            self.approach_pending = False
            self.set_state(MissionState.ERROR)


    def schedule_detection(self):

        delay = self.get_parameter(
            'object_detector_delay'
        ).value

        self.get_logger().info(
            f'Waiting {delay:.1f} seconds before object detection...'
        )

        self.detection_timer = self.create_timer(
            delay, # type: ignore
            self.delayed_detection,
            callback_group=self.default_callback_group
        )


    def delayed_detection(self):

        # Cancel the timer to prevent repeated execution
        self.detection_timer.cancel() # type: ignore
        self.detection_timer = None

        self.detect_objects()


def main(args=None):
    rclpy.init(args=args)

    node = OrchestratorNode(name='orchestrator_node')

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()