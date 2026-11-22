from std_srvs.srv import Trigger

class PerceptionClient:
    def __init__(self, node):
        self.node = node
        
        self.client = self.node.create_client(Trigger, 'perception/detect_all')
        
        
    def detect_objects(self, callback):
        if not self.client.service_is_ready():
            self.node.get_logger().info(
                'Waiting for perception service...'
            )

            if not self.client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warning(
                    'Perception service is unavailable.'
                )
                return False
            
        request = Trigger.Request()

        self.node.get_logger().info(
            'Requesting object detection...'
        )

        future = self.client.call_async(request)

        future.add_done_callback(callback)

        return True
    
    
    def parse_response(self, response):

        detected_objects = []

        for line in response.message.strip().splitlines():

            parts = line.split()

            if len(parts) != 5:
                continue

            color = parts[0]

            if parts[1] == 'not_found':
                self.node.get_logger().info(
                    f'{color} cube not detected.'
                )
                continue

            try:
                position = {
                    'x': float(parts[1]),
                    'y': float(parts[2]),
                    'z': float(parts[3]),
                    'frame_id': parts[4]
                }

            except ValueError:
                self.node.get_logger().warning(
                    f'Invalid detection response: {line}'
                )
                continue

            detected_objects.append({
                'color': color,
                'position': position
            })

        return detected_objects