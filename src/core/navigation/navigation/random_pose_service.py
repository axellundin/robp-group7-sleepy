import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from core_interfaces.srv import MoveTo
import numpy as np
from geometry_msgs.msg import Polygon
from rclpy.qos import QoSProfile, DurabilityPolicy
from core_interfaces.srv import GeofenceCompliance
import time 

class SendGoalClient(Node):
    def __init__(self):
        super().__init__('random_pose_service')
        print("Inne 1")

        self._move_to_client = self.create_client(MoveTo, 'MoveTo')
        while not self._move_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self._goal_list = []
        #self.generate_hardcoded_goals()
        self._goal_nr = 0
    
    def generate_hardcoded_goals(self):
        """
        This function is called to generate a list of hardcoded goals.
        """
        "circle path is active"
        print("circle path is active")

        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        points = 12
        angle_inc = 2*np.pi/points
        for i in range(points):
            angle = -np.pi/2 +i*angle_inc
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = 0.3*np.cos(angle)
            goal_msg.pose.position.y =0.3*(np.sin(angle)+1)
            goal_msg.pose.orientation.z = 0.0 
            goal_msg.pose.orientation.w = 1.0
            goal_list.poses.append(goal_msg)
        request_msg.path = goal_list
        request_msg.enforce_orientation = True
        request_msg.stop_at_goal = True
        request_msg.max_speed = 4.0
        request_msg.max_turn_speed = 0.25
        self.future = self._move_to_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

        # """The following is a path (zigzag) made by yassir to evaluate odometry"""
        # print("zigzag")
        # request_msg = MoveTo.Goal()
        # goal_list = Path()
        # goal_list.header.frame_id = "map"
        # goal_list.header.stamp = self.get_clock().now().to_msg()
        
        # # Alternating left/right turns
        # waypoints = [
        #     (0.5, 0.0), 
        #     (1.0, 0.5),
        #     (1.5, 0.0),  
        #     (2.0, 0.5),  
        #     (2.5, 0.0),  
        #     (3.0, 0.5)  
        # ]

        # for x, y in waypoints:
        #     goal_msg = PoseStamped()
        #     goal_msg.header.frame_id = "map"
        #     goal_msg.header.stamp = self.get_clock().now().to_msg()
        #     goal_msg.pose.position.x = x
        #     goal_msg.pose.position.y = y
        #     goal_msg.pose.orientation.z = 0.0
        #     goal_msg.pose.orientation.w = 1.0
        #     goal_list.poses.append(goal_msg)
        
        # request_msg.path = goal_list
        # request_msg.enforce_orientation = True
        # request_msg.stop_at_goal = True
        # self._send_goal(request_msg, self.goal_response_callback)




    def geofence_callback(self, polygon):
        """ 
        This function is called when the geofence is received.
        It updates the geofence and the bounding box.
        """
        self.get_logger().info('Geofence received: %s' % polygon)
        self.geofence = polygon
        self.bounding_box = [float('inf'), float('inf'), float('-inf'), float('-inf')]
        for point in self.geofence.points:
            self.bounding_box[0] = min(self.bounding_box[0], point.x)
            self.bounding_box[1] = min(self.bounding_box[1], point.y)
            self.bounding_box[2] = max(self.bounding_box[2], point.x)
            self.bounding_box[3] = max(self.bounding_box[3], point.y)

    """
    def rejection_sample(self) -> MoveTo.Goal:
        # This function is called to sample a random pose from the bounding box.
        # It uses rejection sampling to ensure the pose is within the geofence.
        if self.bounding_box is None:
            print("Bounding box not set")
            return None
    
        min_x, min_y, max_x, max_y = self.bounding_box
        found_valid_pose = False
        while not found_valid_pose:
            rx, ry = np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)
            req = GeofenceCompliance.Request()
            req.pose = PoseStamped() 
            req.pose.header.frame_id = "map" 
            req.pose.header.stamp = self.get_clock().now().to_msg()
            req.pose.pose.position.x = rx
            req.pose.pose.position.y = ry
            req.pose.pose.position.z = 0.0
            req.pose.pose.orientation.z = 0.0
            req.pose.pose.orientation.w = 1.0

            if not self.geofence_compliance_client.call_async(req):
                continue

            goal_msg = MoveTo.Goal()
            goal_msg.way_point = req.pose
            goal_msg.enforce_orientation = False
            goal_msg.stop_at_goal = True
            found_valid_pose = True
        return goal_msg
    
    
    
    def _send_goal(self, goal_msg, callback):

        self._move_to_client.wait_for_server()

        self._move_request = self._move_to_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._move_request.add_done_callback(callback)
    
    def test_rejection_sample(self):
         
        # This function is used to test the rejection sampling.
        
        goal_msg = self.rejection_sample()
        print(f"Rejection sampling gives goal: {goal_msg.way_point.pose.position.x}, {goal_msg.way_point.pose.position.y}")

    def send_random_goals_until_stopped(self):
         
        # This is called recursively when the last goal is reached.
        
        goal_msg = self.rejection_sample()
        if goal_msg is None:
            return
        self._send_goal(goal_msg, self.send_random_goal_callback)

    def wait_and_then(self, handle):
        time.sleep(1)
        self.send_random_goals_until_stopped()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: Distance to goal = {feedback.distance_to_goal:.2f} m')
    
    def send_random_goal_callback(self, future):
         
        # Callback for sending random goals until the goal is stopped.
        
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.wait_and_then)

    def goal_response_callback(self, future):
        
        # Callback for sending the next hardcoded goal.
        
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.finish_callback)

    def finish_callback(self, future):
         
        # Callback for sending the next hardcoded goal.
        # This should be called when the last goal is reached.
        
        result = future.result().result
        if result.success:
            self.get_logger().info('Framme!!!!!!!!!!!')
        else:
            self.get_logger().info('Målet misslyckades :(')

        rclpy.shutdown()
"""
def main(args=None):
    rclpy.init(args=args)
    action_client = SendGoalClient()
    # Wait for geofence to be received
    # while action_client.geofence is None:
    #     rclpy.spin_once(action_client)

    # # # Test rejection sampling 
    # for i in range(10):
    #     action_client.test_rejection_sample()

    #action_client.send_random_goals_until_stopped() # Axels version
    action_client.generate_hardcoded_goals() # Hampus version

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()