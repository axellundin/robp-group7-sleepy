import numpy as np
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class MovementComputation:
    def __init__(self, max_speed_forward, max_speed_turn, allow_reverse, dist_to_last_pose, parent_node):
        self.parent_node = parent_node
        #Distance from target when break
        self.dist_threshold = 0.05
        self.slow_down_threshold = 0.4
        self.slow_down_min_speed = 0.1
        self.slow_down_max_speed = max_speed_forward
        self.k_slow_down = (self.slow_down_max_speed - self.slow_down_min_speed)/(self.slow_down_threshold - self.dist_threshold)
        self.m_slow_down = self.slow_down_min_speed - self.k_slow_down*self.dist_threshold
        
        #Tolerance of yaw error at stop pose
        self.yaw_threshold = 2
        self.yaw_threshold = self.yaw_threshold*np.pi/180

        #Maximum difference in speed between two ticks
        self._vel_change_threshold = 0.1

        #Tolerated diff from optimal direction during turning before forward driving
        self._deg_tol_turn = 3 #deg

        #Yaw adjustment speed during turning before forward driving
        self.max_turn = max_speed_turn
        self.min_turn = 0.12
        self.max_turn_threshold = np.pi/6

        self.rad_tol_turn = np.pi*self._deg_tol_turn/180
        self.k_angle = (self.max_turn - self.min_turn)/(self.max_turn_threshold - self.rad_tol_turn)
        self.m_angle = self.min_turn - self.k_angle*self.rad_tol_turn

        #Tolerated diff from optimal direction during forward driving befor stop for yaw adjustment
        self._deg_tol_forward = 45
        
        #Yaw adjustment speed during forward driving
        self.max_turn_forward = max_speed_forward / 2
        self.min_turn_forward = 0.007#0.01
        self.rad_tol_forward = np.pi*self._deg_tol_forward/180
        
        self.k_forward_turn = (self.max_turn_forward - self.min_turn_forward)/self.rad_tol_forward
        self.m_forward_turn = self.min_turn_forward
        

        #Forward speed adjustment during high yaw adjustment while driving forward
        self.max_speed_forward = max_speed_forward
        self.min_speed_forward = 0.0
        self.max_speed_threshold = 0.05 #w
        self.min_speed_threshold = 0.2 #w

        self.k_forward = (self.max_speed_forward - self.min_speed_forward)/(self.max_speed_threshold - self.min_speed_threshold)
        self.m_forward = self.min_speed_forward - self.k_forward*self.min_speed_threshold

        self._move_state = "turn"
        self.start_yaw_ajustment = False

        self._current_vel_x = 0
        self._current_vel_w = 0
        self.allow_reverse_threshold = 0.6
        self.allow_reverse = allow_reverse
        if dist_to_last_pose > self.allow_reverse_threshold:
            self.allow_reverse = False
    

    def compute_vel_from_transform(self, transformed_goal_pose, point_only, stop_at_goal):
        #Calculates the distance to the goal and breaks if the robot are close enough
        dist = np.sqrt(transformed_goal_pose.position.x**2 + transformed_goal_pose.position.y**2)

        #Converting orientation to yaw
        qx = transformed_goal_pose.orientation.x
        qy = transformed_goal_pose.orientation.y
        qz = transformed_goal_pose.orientation.z
        qw = transformed_goal_pose.orientation.w
        yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2))
        if point_only == True:
            yaw = 0

        self.parent_node.get_logger().info(f"dist: {dist} yaw: {yaw}")
        if dist < self.dist_threshold:
            self.start_yaw_ajustment = True
            
        if self.start_yaw_ajustment and abs(yaw) < self.yaw_threshold :
            return True, None

        # ##Feedback sending
        # feedback_msg = MoveTo.Feedback()
        # feedback_msg.distance_to_goal = float(dist)
        # # goal_handle.publish_feedback(feedback_msg)

        self.parent_node.get_logger().info("goal pose: ")
        self.parent_node.get_logger().info(str(transformed_goal_pose.position.x))
        self.parent_node.get_logger().info(str(transformed_goal_pose.position.y))
        self.parent_node.get_logger().info(str(yaw))
        self.parent_node.get_logger().info("Yaw:")
        self.parent_node.get_logger().info(str(yaw))
        self.parent_node.get_logger().info("Dist:")
        self.parent_node.get_logger().info(str(dist))

        angle = np.arctan2(transformed_goal_pose.position.y,transformed_goal_pose.position.x)

        if angle<np.pi/2 and angle>-np.pi/2 or dist < self.dist_threshold*1.2 or dist >self.allow_reverse_threshold:
            self.allow_reverse = False
        self.parent_node.get_logger().info("\nallow_reverse:")
        self.parent_node.get_logger().info(str(self.allow_reverse))
        self.parent_node.get_logger().info("\n\n")
        #Moves towards the goal if the robot isn't close enough
        if dist > self.dist_threshold and self.start_yaw_ajustment == False:
            if self.allow_reverse == True:
                angle = angle - (angle/np.abs(angle))*np.pi
            vel = Twist()

            self.parent_node.get_logger().info("angle: ")
            self.parent_node.get_logger().info(str(angle*180/np.pi))
            #Generates different driving commands based on the angle to the target
            #and whether the robot is moving forward or currently stationary and turning
            if (angle < -self.rad_tol_turn and self._move_state == "turn") or (angle < -self.rad_tol_forward and self._move_state == "forward"):
                self._move_state = "turn"
                self.parent_node.get_logger().info("right")
                vel.linear.x = 0.0
                vel.angular.z = np.max([-self.max_turn, self.k_angle*angle - self.m_angle])
            elif (angle > self.rad_tol_turn and self._move_state == "turn") or (angle > self.rad_tol_forward and self._move_state == "forward"):
                self._move_state = "turn"
                self.parent_node.get_logger().info("left")
                vel.linear.x = 0.0
                vel.angular.z = np.min([self.max_turn, self.k_angle*angle + self.m_angle])
            elif angle < - self.rad_tol_turn:
                self._move_state = "forward"
                self.parent_node.get_logger().info("right and forward")
                vel.angular.z = np.max([-self.max_turn, self.k_forward_turn*angle - self.m_forward_turn])
                vel.linear.x = np.min([self.max_speed_forward, self.k_forward*abs(vel.angular.z) + self.m_forward])
            elif angle > self.rad_tol_turn:
                self._move_state = "forward"
                self.parent_node.get_logger().info("left and forward")
                vel.angular.z = np.min([self.max_turn, self.k_forward_turn*angle + self.m_forward_turn])
                vel.linear.x = np.min([self.max_speed_forward, self.k_forward*abs(vel.angular.z) + self.m_forward])
            else:
                self._move_state = "forward"
                self.parent_node.get_logger().info("Foooooooorward")
                vel.linear.x = self.max_speed_forward
                vel.angular.z = 0.0

        #Turns until the robot's direction matches the desired orientation when the
        #robot has reached the target and the orientation is enforced in the request
        elif abs(yaw) > self.yaw_threshold:
            self._move_state = "turn"
            vel = Twist()
            if yaw < -self.yaw_threshold:
                self.parent_node.get_logger().info("pose fix right")
                vel.linear.x = 0.0
                vel.angular.z = np.max([-self.max_turn, self.k_angle*yaw - self.m_angle])
            elif yaw > self.yaw_threshold:
                self.parent_node.get_logger().info("pose fix left")
                vel.linear.x = 0.0
                vel.angular.z = np.min([self.max_turn, self.k_angle*yaw + self.m_angle])

        #Limits the change in speed if it is too large to prevent damage to the motors
        if vel.linear.x > self._vel_change_threshold + self._current_vel_x or vel.linear.x < self._current_vel_x - self._vel_change_threshold or vel.angular.z > self._vel_change_threshold + self._current_vel_w or vel.angular.z < self._current_vel_w - self._vel_change_threshold:
            dif_x = vel.linear.x -self._current_vel_x
            dif_w = vel.angular.z -self._current_vel_w
            vel.linear.x = self._current_vel_x + self._vel_change_threshold*(dif_x/(abs(dif_x)+abs(dif_w)))
            vel.angular.z = self._current_vel_w + self._vel_change_threshold*(dif_w/(abs(dif_x)+abs(dif_w)))

        #Slows down as the robot approaches the target if stopping at the target is requested
        if dist < self.slow_down_threshold and dist > self.dist_threshold and stop_at_goal:
            self.parent_node.get_logger().info("vel.linear.x")
            self.parent_node.get_logger().info(str(vel.linear.x))
            self.parent_node.get_logger().info("self.k_slow_down*dist + self.m_slow_down")
            self.parent_node.get_logger().info(str(self.k_slow_down*dist + self.m_slow_down))
            vel.linear.x = np.min([vel.linear.x, self.k_slow_down*dist + self.m_slow_down])


        #Uppdating Current velocity
        self._current_vel_x = vel.linear.x
        self._current_vel_w = vel.angular.z
        if self.allow_reverse == True and dist > self.dist_threshold:
            vel.linear.x = -vel.linear.x
        self.parent_node.get_logger().info("\nHastigheter:")
        self.parent_node.get_logger().info(str(vel))
        return False, vel
        