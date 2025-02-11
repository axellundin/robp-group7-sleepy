#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core_interfaces.msg import DetectedMsg

class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_gen')
        self.get_logger().info('Map Generator node has been started')
        self.objects = []
        self.boxes = []
        self.path = "/home/sleepy/robp-group7-sleepy/"
        self.has_changes = False  # Track if changes occurred

        self.detected_objects_subscriber = self.create_subscription(
            DetectedMsg,
            'detected',
            self.detected_callback,
            10
        )
        self.time_between_updates = 10.0  # seconds
        self.last_update = self.get_clock().now()

        # Create timer that checks for updates every second
        self.timer = self.create_timer(1.0, self.check_and_generate_map)

    def detected_callback(self, msg: DetectedMsg):
        self.get_logger().info('Received detected objects')
        if self.objects != msg.objects or self.boxes != msg.boxes:
            self.has_changes = True
        self.objects = msg.objects
        self.boxes = msg.boxes

    def check_and_generate_map(self):
        current_time = self.get_clock().now()
        time_since_last_update = (current_time - self.last_update).nanoseconds / 1e9

        if self.has_changes and time_since_last_update >= self.time_between_updates:
            self.generate_map()
            self.last_update = current_time
            self.has_changes = False
            self.get_logger().info('Map generated due to changes')

    def generate_map(self):
        path = self.path + "map.txt"
        with open(path, 'w') as f:
            for object in self.objects:
                f.write(f'1 {object.pose.position.x} {object.pose.position.y} 0\n')
            for box in self.boxes:
                f.write(f'B {box.pose.position.x} {box.pose.position.y} {box.pose.orientation.z}\n')

def main(args=None):
    rclpy.init(args=args)
    
    node = MapGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
