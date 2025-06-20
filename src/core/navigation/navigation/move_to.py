import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from core_interfaces.srv import MoveTo
from geometry_msgs.msg import Twist
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import tf2_geometry_msgs
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from navigation.move_to_logic import MovementComputation
from _thread import start_new_thread 
from builtin_interfaces.msg import Duration
from rclpy.logging import set_logger_level

class Move_to(Node):

    def __init__(self):
        super().__init__('move_to')
        print("inne move 7")

        self.movement_computation = None
        self.should_continue = True
        self.all_done = True 


        group = ReentrantCallbackGroup()
        self.server = self.create_service(MoveTo, "MoveTo", self.move_callback, callback_group=group)

        # Creating publisher for motor drive
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=group)

        #creating buffer and transform listener
        buffer_size = rclpy.duration.Duration(seconds=10.0) 
        self.tf_buffer = Buffer(cache_time=buffer_size)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.alive = True  
        self.last_id = 0

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.velocity_pub = self.create_publisher(Twist, '/current_velocity', 10)
        self.next_waypoint_pub = self.create_publisher(PoseStamped, '/next_waypoint', 10)
        self.should_stop_sub = self.create_subscription(Bool, '/should_stop', self.should_stop_callback, 10)

        self.velocity_last_update = self.get_clock().now() 
        self.velocity_update_interval = 0.1
        self.should_stop = False
        self.has_stopped = False

        start_new_thread(self.spin_thread, ())

    def spin_thread(self):
        while self.alive:
            rclpy.spin_once(self, timeout_sec=0.001)

    def should_stop_callback(self, msg):
        self.should_stop = msg.data

    def move_callback(self, request, response):
        self.get_logger().debug(f"Move to callback with poses {[(pose.pose.position.x, pose.pose.position.y) for pose in request.path.poses]}")
        self.should_continue = False
        self.last_id += 1 
        my_id = self.last_id

        while not self.all_done:
            self.executor.spin_once(timeout_sec=0.01)
        if not my_id == self.last_id:
            self.get_logger().info(f"{"----"*100}Move to {my_id} cancelled{"----"*100}")
            return response
        self.all_done = False
        self.should_continue = True
        #Fetching request
        pose_list = request.path.poses
        point_only_request = request.enforce_orientation
        stop_at_goal_request = request.stop_at_goal
        dist_threshold = request.threshold

        last_pose = self.transform(pose_list[-1])
        dist_to_last_pose = np.sqrt(last_pose.position.x**2 + last_pose.position.y**2)
        self.movement_computation = MovementComputation(request.max_speed, request.max_turn_speed, request.allow_reverse, dist_to_last_pose, dist_threshold, self)

        # determines whether reverse driving should be allowed

        for goal_pose_index in range(len(pose_list)):
            if not self.should_continue:
                break
            goal_pose = pose_list[goal_pose_index] 
            self.get_logger().debug(f"Using waypoint {goal_pose_index} at {goal_pose}")
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.next_waypoint_pub.publish(goal_pose)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "move_to"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            point = Point()
            point.x = goal_pose.pose.position.x
            point.y = goal_pose.pose.position.y
            point.z = 0.0
            marker.pose = goal_pose.pose
            marker.points.append(point)
            marker.scale.x = 0.1 
            marker.scale.y = 0.1 
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.lifetime = Duration(seconds=0)

            point_only = True
            stop_at_goal = False

            # If at end of path
            if goal_pose_index == len(pose_list)-1:
                point_only = not(point_only_request)
                stop_at_goal = stop_at_goal_request
        
            if not point_only:
                marker.type = Marker.ARROW 
                marker.pose = PoseStamped().pose 
                point = Point() 
                angle_from_quaternion = np.arctan2( 2 * (goal_pose.pose.orientation.w * goal_pose.pose.orientation.z + goal_pose.pose.orientation.x * goal_pose.pose.orientation.y), 1 - 2 * (goal_pose.pose.orientation.y**2 + goal_pose.pose.orientation.z**2))
                point.x = goal_pose.pose.position.x + 0.2 * np.cos(angle_from_quaternion)
                point.y = goal_pose.pose.position.y + 0.2 * np.sin(angle_from_quaternion)
                point.z = 0.0
                marker.points.append(point)
                marker.scale.x = 0.02
                marker.scale.y = 0.05
                marker.scale.z = 0.05

            self.marker_pub.publish(marker)
            
            if stop_at_goal:
                self.movement_computation._move_state = "turn"
            else:
                self.movement_computation._move_state = "forward"
            self.movement_computation.start_yaw_ajustment = False
            #Loop until request is cancel or goal is achieved''

            # # handle small adjustments
            # tmp = self.transform(goal_pose)
            # start_dist_to_goal = np.sqrt(tmp.position.x**2 + tmp.position.y**2)
            # qx = tmp.orientation.x
            # qy = tmp.orientation.y
            # qz = tmp.orientation.z
            # qw = tmp.orientation.w
            # start_yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2))
            # if start_dist_to_goal <= dist_threshold and goal_pose_index == len(pose_list)-1 and point_only == True:
            #     self.movement_computation.dist_threshold = 0.015
            #     if start_dist_to_goal <= 0.015:
            #         self.get_logger().warning('too close waypoint')
            #         response.success = True
            #         self.all_done = True
            #         return response
            # if start_dist_to_goal <= dist_threshold and start_yaw <= self.movement_computation.yaw_threshold and goal_pose_index == len(pose_list)-1 and point_only == False:
            #     self.get_logger().warning('too small yaw adjustment waypoint')
            #     response.success = True
            #     self.all_done = True
            #     return response

            while self.should_continue: 
                if self.should_stop: 
                    if not self.has_stopped: 
                        self.ICP_calibration_sequence()
                        self.has_stopped = True 
                    time.sleep(0.01)
                    continue
                self.has_stopped = False 
                #Transforming goal pose
                # frames = self.tf_buffer.all_frames_as_yaml()
                # self.get_logger().info(f'Available TF frames:\n{frames}')
                transformed_goal_pose = self.transform(goal_pose)
                success, vel = self.movement_computation.compute_vel_from_transform(transformed_goal_pose, point_only, stop_at_goal)

                if success:
                    break

                #Publishing velocity command 
                self._pub_vel.publish(vel)

                current_time_in_seconds = self.get_clock().now().to_msg().sec
                last_time_in_seconds = self.velocity_last_update.to_msg().sec
                if current_time_in_seconds - last_time_in_seconds > self.velocity_update_interval:
                    msg = Twist()
                    msg.linear.x = self.movement_computation._current_vel_x
                    msg.angular.z = self.movement_computation._current_vel_w
                    self.velocity_pub.publish(msg)
                    self.velocity_last_update = self.get_clock().now()

        if not self.should_continue: 
            self.get_logger().debug("Move to cancelled")
            response.success = False 
            self.all_done = True
            return response

        #Publishing stop velocity command if stopping at the target is requested
        if stop_at_goal:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self._pub_vel.publish(vel)

        response.success = True
        print("move_to finished")
        self.all_done = True
        return response
    
    def get_current_pose(self):
        current_pose = PoseStamped()
        current_pose.header.frame_id = "map"
        current_pose.pose.position.x = 0.0
        current_pose.pose.position.y = 0.0
        current_pose.pose.position.z = 0.0
        return self.transform(current_pose)

    def ICP_calibration_sequence(self):
        # max_speed = max(self.movement_computation._current_vel_x, 0.15)
        # max_turn_speed = max(self.movement_computation._current_vel_w, 0.15)
        # allow_reverse = False
        # dist_to_last_pose = 100000
        # dist_threshold = 0.03
        # temp_movement_computation = MovementComputation(max_speed, max_turn_speed, allow_reverse, dist_to_last_pose, dist_threshold, self)
        # temp_movement_computation.start_yaw_ajustment = False
        # temp_movement_computation._move_state = "forward" 
        # current_pose = self.get_current_pose() # This is an inverted transform
        # goal_pose = PoseStamped() 
        # goal_pose.header.frame_id = "map"
        # dist_to_move = self.movement_computation._current_vel_x / 3 
        # current_heading = np.arctan2(2 * (current_pose.orientation.w * current_pose.orientation.z + current_pose.orientation.x * current_pose.orientation.y), 1 - 2 * (current_pose.orientation.y**2 + current_pose.orientation.z**2))
        # goal_pose.pose.position.x = -current_pose.position.x + dist_to_move * np.cos(current_heading)
        # goal_pose.pose.position.y = -current_pose.position.y + dist_to_move * np.sin(current_heading)
        # goal_pose.pose.orientation.x = 0.0
        # goal_pose.pose.orientation.y = 0.0
        # goal_pose.pose.orientation.z = 0.0
        # goal_pose.pose.orientation.w = 1.0
        # point_only = False
        # stop_at_goal = True
        # while True: 
        #     transformed_goal_pose = self.transform(goal_pose)
        #     success, vel = temp_movement_computation.compute_vel_from_transform(transformed_goal_pose, point_only, stop_at_goal)

        #     if success:
        #         break

        #     #Publishing velocity command 
        #     self._pub_vel.publish(vel)

        #     current_time_in_seconds = self.get_clock().now().to_msg().sec
        #     last_time_in_seconds = self.velocity_last_update.to_msg().sec
        #     if current_time_in_seconds - last_time_in_seconds > self.velocity_update_interval:
        #         msg = Twist()
        #         msg.linear.x = temp_movement_computation._current_vel_x
        #         msg.angular.z = temp_movement_computation._current_vel_w
        #         self.velocity_pub.publish(msg)
        #         self.velocity_last_update = self.get_clock().now()

        # if stop_at_goal:
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self._pub_vel.publish(vel)
        time.sleep(.5)
        
    def transform(self,goal_pose):
        #self.get_logger().info("transforming")
        _time = rclpy.time.Time(seconds=0)#self.get_clock().now().to_msg()#rclpy.time.Time()

        try:
            t = self.tf_buffer.lookup_transform(
            "base_link",
            goal_pose.header.frame_id,
            _time,
            #timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"base_link"} to {goal_pose.header.frame_id}: {ex}')
            return
        #print(t)
        return tf2_geometry_msgs.do_transform_pose(goal_pose.pose, t)
        
def main():
    rclpy.init()
    node = Move_to()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.alive = False
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# import rclpy.time
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# from core_interfaces.srv import MoveTo
# from tf2_ros import TransformException
# import numpy as np
# from robp_interfaces.msg import DutyCycles
# from geometry_msgs.msg import PoseStamped
# import tf2_geometry_msgs
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# import time

# class Move_to(Node):

#     def __init__(self):
#         super().__init__('move_to')
#         print("inne move 4")

#         #Distance from target when break
#         self.dist_threshold = 0.05
#         self.slow_down_threshold = 0.4
#         self.slow_down_min_speed = 0.085
#         self.slow_down_max_speed = 0.4
#         self.k_slow_down = (self.slow_down_max_speed - self.slow_down_min_speed)/(self.slow_down_threshold - self.dist_threshold)
#         self.m_slow_down = self.slow_down_min_speed - self.k_slow_down*self.dist_threshold
        
#         #Tolerance of yaw error at stop pose
#         self.yaw_threshold = 2
#         self.yaw_threshold = self.yaw_threshold*np.pi/180

#         #Maximum difference in speed between two ticks
#         self._vel_change_threshold = 0.2

#         #Tolerated diff from optimal direction during turning before forward driving
#         self._deg_tol_turn = 3 #deg

#         #Yaw adjustment speed during turning before forward driving
#         self.max_turn = 0.25
#         self.min_turn = 0.09
#         self.max_turn_threshold = np.pi/4

#         self.rad_tol_turn = np.pi*self._deg_tol_turn/180
#         self.k_angle = (self.max_turn - self.min_turn)/(self.max_turn_threshold - self.rad_tol_turn)
#         self.m_angle = self.min_turn - self.k_angle*self.rad_tol_turn

#         #Tolerated diff from optimal direction during forward driving befor stop for yaw adjustment
#         self._deg_tol_forward = 45
        
#         #Yaw adjustment speed during forward driving
#         self.max_turn_forward = 0.2#0.15#0.1#0.075
#         self.min_turn_forward = 0.008#0.01
#         self.rad_tol_forward = np.pi*self._deg_tol_forward/180
        
#         self.k_forward_turn = (self.max_turn_forward - self.min_turn_forward)/self.rad_tol_forward
#         self.m_forward_turn = self.min_turn_forward
        

#         #Forward speed adjustment during high yaw adjustment while driving forward
#         self.max_speed_forward = 0.4
#         self.min_speed_forward = 0.0
#         self.max_speed_threshold = 0.05 #w
#         self.min_speed_threshold = 0.2 #w

#         self.k_forward = (self.max_speed_forward - self.min_speed_forward)/(self.max_speed_threshold - self.min_speed_threshold)
#         self.m_forward = self.min_speed_forward - self.k_forward*self.min_speed_threshold

#         self._move_state = "turn"
#         self.executor = None

#         group = ReentrantCallbackGroup()
#         #Creating action server executing the path 
#         self.move_service = self.create_service(MoveTo, "MoveTo", self.move_callback, callback_group=group)

#         # Creating publisher for motor drive
#         self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
#         self._current_vel_x = 0
#         self._current_vel_w = 0

#         #creating buffer and transform listener
#         buffer_size = rclpy.duration.Duration(seconds=10.0) 
#         self.tf_buffer = Buffer(cache_time=buffer_size)
#         self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

#     def move_callback(self, request, response):
#         self.get_logger().info('Move_to has been called')
#         #Fetching request
#         pose_list = request.path.poses
#         point_only_request = request.enforce_orientation
#         stop_at_goal_request = request.stop_at_goal
#         self.max_speed_forward = request.max_speed
#         self.slow_down_threshold = self.max_speed_forward
#         self.max_turn_forward = self.max_speed_forward/2
#         self.max_turn = request.max_turn_speed
#         for goal_pose_index in range(len(pose_list)):
#             goal_pose = pose_list[goal_pose_index]

#             if goal_pose_index != len(pose_list)-1:
#                 point_only = True
#                 stop_at_goal = False
#             else:
#                 point_only = not(point_only_request)
#                 stop_at_goal = stop_at_goal_request

#             if stop_at_goal:
#                 self._move_state = "turn"
#             else:
#                 self._move_state = "forward"

#             #Loop until request is cancel or goal is achieved
#             while True:
#                 #Transforming goal pose
#                 # frames = self.tf_buffer.all_frames_as_yaml()
#                 # self.get_logger().info(f'Available TF frames:\n{frames}')
#                 transformed_goal_pose = self.transform(goal_pose)

#                 #Calculates the distance to the goal and breaks if the robot are close enough
#                 dist = np.sqrt(transformed_goal_pose.position.x**2 + transformed_goal_pose.position.y**2)

#                 #Converting orientation to yaw
#                 qx = transformed_goal_pose.orientation.x
#                 qy = transformed_goal_pose.orientation.y
#                 qz = transformed_goal_pose.orientation.z
#                 qw = transformed_goal_pose.orientation.w
#                 if point_only == True:
#                     yaw = 0
#                 else:
#                     yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2))

#                 if dist < self.dist_threshold and abs(yaw) < self.yaw_threshold :
#                     break

#                 ##Feedback sending
#                 # feedback_msg = MoveTo.Feedback()
#                 # feedback_msg.distance_to_goal = float(dist)
#                 # goal_handle.publish_feedback(feedback_msg)

#                 print("goal pose: ")
#                 print(transformed_goal_pose.position.x)
#                 print(transformed_goal_pose.position.y)
#                 print(yaw)
#                 print("Yaw:")
#                 print(yaw)
#                 print("Dist:")
#                 print(dist)
                
#                 #Moves towards the goal if the robot isn't close enough
#                 if dist > self.dist_threshold:
#                     angle = np.arctan2(transformed_goal_pose.position.y,transformed_goal_pose.position.x)

#                     vel = Twist()

#                     print("angle: ")
#                     print(angle*180/np.pi)
#                     #Generates different driving commands based on the angle to the target
#                     #and whether the robot is moving forward or currently stationary and turning
#                     if (angle < -self.rad_tol_turn and self._move_state == "turn") or (angle < -self.rad_tol_forward and self._move_state == "forward"):
#                         self._move_state = "turn"
#                         print("right")
#                         vel.linear.x = 0.0
#                         vel.angular.z = np.max([-self.max_turn, self.k_angle*angle - self.m_angle])
#                     elif (angle > self.rad_tol_turn and self._move_state == "turn") or (angle > self.rad_tol_forward and self._move_state == "forward"):
#                         self._move_state = "turn"
#                         print("left")
#                         vel.linear.x = 0.0
#                         vel.angular.z = np.min([self.max_turn, self.k_angle*angle + self.m_angle])
#                     elif angle < - self.rad_tol_turn:
#                         self._move_state = "forward"
#                         print("right and forward")
#                         vel.angular.z = np.max([-self.max_turn, self.k_forward_turn*angle - self.m_forward_turn])
#                         vel.linear.x = np.min([self.max_speed_forward, self.k_forward*abs(vel.angular.z) + self.m_forward])
#                     elif angle > self.rad_tol_turn:
#                         self._move_state = "forward"
#                         print("left and forward")
#                         vel.angular.z = np.min([self.max_turn, self.k_forward_turn*angle + self.m_forward_turn])
#                         vel.linear.x = np.min([self.max_speed_forward, self.k_forward*abs(vel.angular.z) + self.m_forward])
#                     else:
#                         self._move_state = "forward"
#                         print("Foooooooorward")
#                         vel.linear.x = self.max_speed_forward
#                         vel.angular.z = 0.0

#                 #Turns until the robot's direction matches the desired orientation when the
#                 #robot has reached the target and the orientation is enforced in the request
#                 elif abs(yaw) > self.yaw_threshold:
#                     self._move_state = "turn"
#                     vel = Twist()
#                     if yaw < -self.yaw_threshold:
#                         print("pose fix right")
#                         vel.linear.x = 0.0
#                         vel.angular.z = np.max([-self.max_turn, self.k_angle*yaw - self.m_angle])
#                     elif yaw > self.rad_tol_turn:
#                         print("pose fix left")
#                         vel.linear.x = 0.0
#                         vel.angular.z = np.min([self.max_turn, self.k_angle*yaw + self.m_angle])

#                 #Limits the change in speed if it is too large to prevent damage to the motors
#                 if vel.linear.x > self._vel_change_threshold + self._current_vel_x or vel.linear.x < self._current_vel_x - self._vel_change_threshold or vel.angular.z > self._vel_change_threshold + self._current_vel_w or vel.angular.z < self._current_vel_w - self._vel_change_threshold:
#                     dif_x = vel.linear.x -self._current_vel_x
#                     dif_w = vel.angular.z -self._current_vel_w
#                     vel.linear.x = self._current_vel_x + self._vel_change_threshold*(dif_x/(abs(dif_x)+abs(dif_w)))
#                     vel.angular.z = self._current_vel_w + self._vel_change_threshold*(dif_w/(abs(dif_x)+abs(dif_w)))

#                 #Slows down as the robot approaches the target if stopping at the target is requested
#                 if dist < self.slow_down_threshold and stop_at_goal:
#                     # print("vel.linear.x")
#                     # print(vel.linear.x)
#                     # print("self.k_slow_down*dist + self.m_slow_down")
#                     # print(self.k_slow_down*dist + self.m_slow_down)
#                     vel.linear.x = np.min([vel.linear.x, self.k_slow_down*dist + self.m_slow_down])
                
#                 # Uppdating Current velocity
#                 self._current_vel_x = vel.linear.x
#                 self._current_vel_w = vel.angular.z

#                 #Publishing velocity command 
#                 self._pub_vel.publish(vel)
        
#         self.get_logger().info('Finished move callback')
#         #Publishing stop velocity command if stopping at the target is requested
#         if stop_at_goal:
#             vel = Twist()
#             vel.linear.x = 0.0
#             vel.angular.z = 0.0
#             self._pub_vel.publish(vel)
                
#         #Result handeling

#         response.success = True
#         self.get_logger().info('Returning response')
#         return response
    
    
#     def transform(self,goal_pose):
#         #self.get_logger().info('Starting transform')
#         # self.executor.spin_once(timeout_sec=0.001)

#         _time = rclpy.time.Time(seconds=0)

#         try:
#             time_before = time.time()
#             #self.get_logger().info('Before lookup_transform')
#             t = self.tf_buffer.lookup_transform(
#                 "base_link",
#                 goal_pose.header.frame_id,
#                 _time,
#             )
#             time_after = time.time()
#             time_diff = time_after - time_before
#             self.get_logger().info(f'Time taken to transform: {time_diff} seconds')
#             #self.get_logger().info('After lookup_transform')
#             transformed = tf2_geometry_msgs.do_transform_pose(goal_pose.pose, t)
#             #self.get_logger().info('Transform complete')
#             return transformed
#         except TransformException as ex:
#             #self.get_logger().info(
#             #    f'Could not transform {"base_link"} to {goal_pose.header.frame_id}: {ex}')
#             return None 
        
# def main():
#     rclpy.init()
#     node = Move_to()
#     try:

#         ex = MultiThreadedExecutor() 
#         node.executor = ex
#         ex.add_node(node)
#         ex.spin()
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





####With direct motor controll####
"""
            #Robot move pub
            print(self.current_duty_cycle)
            angle = np.arctan2(transformed_goal_pose.position.y,transformed_goal_pose.position.x)
            deg_tol = 5
            rad_tol = np.pi*deg_tol/180
            motor_msg = DutyCycles()
            motor_msg.header.stamp = self.get_clock().now().to_msg()
            motor_msg.header.frame_id = "motors"
            if angle < -rad_tol:
                motor_msg.duty_cycle_right = -0.1
                motor_msg.duty_cycle_left = 0.1
            elif angle > rad_tol:
                motor_msg.duty_cycle_right = 0.1
                motor_msg.duty_cycle_left = -0.1
            else:
                motor_msg.duty_cycle_right = 0.1
                motor_msg.duty_cycle_left = 0.1

            self._pub_motor.publish(motor_msg)

        motor_msg = DutyCycles()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.header.frame_id = "motors"

        self._pub_motor.publish(motor_msg)
        result = MoveTo.Result()
        result.success = True
        goal_handle.succeed()
        return result"""