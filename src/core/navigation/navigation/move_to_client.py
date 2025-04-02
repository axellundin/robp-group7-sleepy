import rclpy
from rclpy.node import Node
from core_interfaces.srv import MoveTo
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import sys
import numpy as np

class MoveToClient(Node):
    def __init__(self):
        super().__init__('move_to_client')
        self.client = self.create_client(MoveTo, 'MoveTo')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveTo service not available, waiting...')
    
    def send_request(self, path, enforce_orientation=True, allow_reverse=False, 
                   stop_at_goal=True, max_speed=0.5, max_turn_speed=1.0):
        """
        Send a request to the MoveTo service
        
        Parameters:
        -----------
        path : nav_msgs.msg.Path
            Path to follow
        enforce_orientation : bool
            Whether to enforce orientation at the goal
        allow_reverse : bool
            Whether to allow reverse motion
        stop_at_goal : bool
            Whether to stop at the goal position
        max_speed : float
            Maximum linear speed
        max_turn_speed : float
            Maximum angular speed
        
        Returns:
        --------
        Future object
        """
        request = MoveTo.Request()
        request.path = path
        request.enforce_orientation = enforce_orientation
        request.allow_reverse = allow_reverse
        request.stop_at_goal = stop_at_goal
        request.max_speed = max_speed
        request.max_turn_speed = max_turn_speed
        
        self.get_logger().info('Sending MoveTo request')
        return self.client.call_async(request)
    
    def create_simple_path(self, x, y, theta=0.0, frame_id='map'):
        """
        Create a simple path with a single pose
        
        Parameters:
        -----------
        x : float
            X coordinate
        y : float
            Y coordinate
        theta : float
            Orientation in radians
        frame_id : str
            Frame ID
            
        Returns:
        --------
        nav_msgs.msg.Path
        """
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(theta / 2.0)
        pose.pose.orientation.w = np.cos(theta / 2.0)
        
        path.poses.append(pose)
        return path

def main(args=None):
    rclpy.init(args=args)
    
    move_to_client = MoveToClient()
    
    # Example usage: Move to position (1.0, 1.0) with orientation 0.0 radians
    path = move_to_client.create_simple_path(2.0, 0.0, 0.0)
    future = move_to_client.send_request(path)
    
    rclpy.spin_until_future_complete(move_to_client, future)
    
    if future.result() is not None:
        response = future.result()
        move_to_client.get_logger().info(f'Result: {"Success" if response.success else "Failed"}')
    else:
        move_to_client.get_logger().error('Service call failed')
    
    move_to_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 