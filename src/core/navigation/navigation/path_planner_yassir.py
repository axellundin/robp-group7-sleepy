import rclpy
import numpy as np
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
import random

from core_interfaces.action import PathPlanner
from core_interfaces.srv import GeofenceCompliance
from core_interfaces.action import MoveTo

class PathPlannerActionServer(Node):

    def __init__(self):
        super().__init__('path_planner_action_server')
        print("Starting path planner")
        self._action_server = ActionServer(
            self,
            PathPlanner,
            'path_planner',
            self.execute_callback,
            cancel_callback=self.cancel_callback)

        # Make a action client to move_to
        self.move_to_client = ActionClient(self, MoveTo, 'move_to')

        self.dist_threshold = 0.15

    def generate_random_pose(self) -> PoseStamped:
        """Generate a random PoseStamped within 1x1 square"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = random.uniform(0.0, 1.0)
        pose_stamped.pose.position.y = random.uniform(0.0, 1.0)
        pose_stamped.pose.position.z = 0.0
        return pose_stamped

    def generate_waypoints_square(self) -> list[PoseStamped]: 
        """Generate a list of waypoints"""
        poses = [ {
            "x": 0.0,
            "y": 0.0,
            "th": 0.0
        },
        {
            "x": 1.0,
            "y": 0.0,
            "th": -np.pi/2
        },
        {
            "x": 1.0,
            "y": 1.0,
            "th": np.pi
        },
        {
            "x": 0.0,
            "y": 1.0,
            "th": np.pi/2
        }
        ]
        poses = poses + poses 
        waypoints = []
        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = pose["x"]
            pose_stamped.pose.position.y = pose["y"]
            pose_stamped.pose.orientation.z = pose["th"]
            waypoints.append(pose_stamped)
        return waypoints

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Generate 5 random waypoints
        waypoints = self.generate_waypoints_square()
        
        for waypoint in waypoints:
            # Start a request to move_to
            goal = MoveTo.Goal()
            goal.way_point = waypoint
            goal.enforce_orientation = False
            
            # Wait for server and send goal
            self.move_to_client.wait_for_server()
            print(f"Sending goal to {waypoint}")
            future = self.move_to_client.send_goal_async(goal)
            
            # Wait for goal response
            rclpy.spin_until_future_complete(self, future)
            goal_handle_move = future.result()
            
            if goal_handle_move.accepted:
                # Wait for the action to complete
                result_future = goal_handle_move.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
            
            if goal_handle.is_cancel_requested:
                break

        result = PathPlanner.Result()
        result.success = True
        goal_handle.succeed()
        return result
    
    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback}')

    def cancel_callback(self, goal_handle):
        """Callback that gets called when the action client requests a goal cancellation"""
        self.get_logger().info('Received cancel request')
        # Cancel the current move_to action if it's active
        if hasattr(self, 'move_to_client') and self.move_to_client.is_active():
            self.move_to_client.cancel_goal()
        return True  # Must return True to indicate the cancellation was accepted

def main(args=None):
    rclpy.init(args=args)

    path_planner_action_server = PathPlannerActionServer()

    rclpy.spin(path_planner_action_server)


if __name__ == '__main__':
    main()