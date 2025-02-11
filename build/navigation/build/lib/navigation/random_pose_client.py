import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from core_interfaces.action import MoveTo 
import numpy as np

class SendGoalClient(Node):
    def __init__(self):
        super().__init__('random_pose_client')
        print("Inne")

        self._move_to_client = ActionClient(self, MoveTo, 'move_to')
        self._goal_list = []
        points = 20
        angle_inc = 2*np.pi/points
        for i in range(points):
            angle = -np.pi/2 +i*angle_inc
            goal_msg = MoveTo.Goal()
            goal_msg.enforce_orientation = False
            goal_msg.stop_at_goal = False
            goal_msg.way_point = PoseStamped()
            goal_msg.way_point.header.frame_id = "map"
            goal_msg.way_point.header.stamp = self.get_clock().now().to_msg()
            goal_msg.way_point.pose.position.x = np.cos(angle)
            goal_msg.way_point.pose.position.y =np.sin(angle)+1
            goal_msg.way_point.pose.orientation.z = 1.0 
            goal_msg.way_point.pose.orientation.w = 0.0
            self._goal_list.append(goal_msg)
        self._goal_nr = 0

    def send_goal(self):

        self._move_to_client.wait_for_server()
        goal_msg = self._goal_list[self._goal_nr]
        print(goal_msg.way_point.pose.position.x)
        print(goal_msg.way_point.pose.position.y)

        self._move_request = self._move_to_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._move_request.add_done_callback(self.goal_response_callback)
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info(f'Feedback: Distance to goal = {feedback.distance_to_goal:.2f} m')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Målet slutfört med framgång!')
        else:
            self.get_logger().info('Målet misslyckades.')

        self._goal_nr += 1
        if self._goal_nr < len(self._goal_list):
            self.send_goal()
        else:
            self.get_logger().info('Alla mål är slutförda.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = SendGoalClient()
    

    action_client.send_goal()
    

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
