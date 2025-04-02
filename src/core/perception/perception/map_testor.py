import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool
from .map import MapNode
import time

class TestMapNode(Node):
    def __init__(self):
        super().__init__('map_test_node')  # Initialize test node
        
        # Initialize MapNode instance for testing
        self.map_node = MapNode()  # Creating a 5x5 grid_map for simplicity
        self.map_node.grid_map[2, 2] = -1  # Marking some unexplored areas as -1
        self.map_node.grid_map[3, 3] = -1  # Another unexplored cell
        self.map_node.grid_map[1, 1] = 100  # Another unexplored cell
        
        self.publish_messages()

    def publish_messages(self):
        robot_pose = (20, 20, 0) 
        time.sleep(1)
        updated_grid = self.map_node.add_explored_area(robot_pose)
        self.get_logger().info(f"Updated grid map:\n{updated_grid}")
        self.map_node.publish_map()
        time.sleep(3)

        robot_pose = (20, 20, 90) 
        time.sleep(1)
        updated_grid = self.map_node.add_explored_area(robot_pose)
        self.get_logger().info(f"Updated grid map:\n{updated_grid}")
        self.map_node.publish_map()
        time.sleep(3)

        robot_pose = (20, 20, 180) 
        time.sleep(1)
        updated_grid = self.map_node.add_explored_area(robot_pose)
        self.get_logger().info(f"Updated grid map:\n{updated_grid}")
        self.map_node.publish_map()
        time.sleep(3)

        robot_pose = (20, 20, 270) 
        time.sleep(1)
        updated_grid = self.map_node.add_explored_area(robot_pose)
        self.get_logger().info(f"Updated grid map:\n{updated_grid}")
        self.map_node.publish_map()
        time.sleep(3)

def main():
    rclpy.init()  # Initialize ROS2
    test_node = TestMapNode()  # Create a test node instance
    try:
        rclpy.spin(test_node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()  # Cleanup node
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    main()
