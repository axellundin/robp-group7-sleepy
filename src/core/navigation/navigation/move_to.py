import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from core_interfaces.action import MoveTo
from geometry_msgs.msg import Twist
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import time


class Move_to(Node):

    def __init__(self):
        super().__init__('move_to')
        print("inne move")


        #Distance from target when break
        self.dist_threshold = 0.05
        self.slow_down_threshold = 0.2
        self.slow_down_min_speed = 0.085
        self.slow_down_max_speed = 0.3
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
        self.max_turn = 0.2
        self.min_turn = 0.09
        self.max_turn_threshold = np.pi/4

        self.rad_tol_turn = np.pi*self._deg_tol_turn/180
        self.k_angle = (self.max_turn - self.min_turn)/(self.max_turn_threshold - self.rad_tol_turn)
        self.m_angle = self.min_turn - self.k_angle*self.rad_tol_turn

        #Tolerated diff from optimal direction during forward driving befor stop for yaw adjustment
        self._deg_tol_forward = 18
        
        #Yaw adjustment speed during forward driving
        self.max_turn_forward = 0.2#0.15#0.1#0.075
        self.min_turn_forward = 0.007#0.01
        self.rad_tol_forward = np.pi*self._deg_tol_forward/180
        
        self.k_forward_turn = (self.max_turn_forward - self.min_turn_forward)/self.rad_tol_forward
        self.m_forward_turn = self.min_turn_forward
        

        #Forward speed adjustment during high yaw adjustment while driving forward
        self.max_speed_forward = 0.3
        self.min_speed_forward = 0.0
        self.max_speed_threshold = 0.1 #w
        self.min_speed_threshold = 0.2 #w

        self.k_forward = (self.max_speed_forward - self.min_speed_forward)/(self.max_speed_threshold - self.min_speed_threshold)
        self.m_forward = self.min_speed_forward - self.k_forward*self.min_speed_threshold

        self._move_state = "turn"

        #Creating action server executing the path 
        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            self.move_callback,
            cancel_callback=self.cancel_callback
        )

        #Creating publisher for motor drive
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._current_vel_x = 0
        self._current_vel_w = 0

        #creating buffer and transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def move_callback(self, goal_handle):
        #Fetching request
        goal_pose = goal_handle.request.way_point
        point_only = goal_handle.request.enforce_orientation
        stop_at_goal = goal_handle.request.stop_at_goal

        if stop_at_goal:
            self._move_state = "turn"
        else:
            self._move_state = "forward"

        while not goal_handle.is_cancel_requested:

            transformed_goal_pose = self.transform(goal_pose)
            dist = np.sqrt(transformed_goal_pose.position.x**2 + transformed_goal_pose.position.y**2)
            qx = transformed_goal_pose.orientation.x
            qy = transformed_goal_pose.orientation.y
            qz = transformed_goal_pose.orientation.z
            qw = transformed_goal_pose.orientation.w
            if point_only == False:
                yaw = 0
            else:
                yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2))

            if dist < self.dist_threshold and abs(yaw) < self.yaw_threshold :
                break
            print("goal pose: ")
            print(transformed_goal_pose.position.x)
            print(transformed_goal_pose.position.y)
            print(yaw)
            ##Feedback sending
            feedback_msg = MoveTo.Feedback()
            feedback_msg.distance_to_goal = float(dist)
            goal_handle.publish_feedback(feedback_msg)
            print("Yaw:")
            print(yaw)
            print("Dist:")
            print(dist)
            
            #Robot move pub
            if dist > self.dist_threshold:
                angle = np.arctan2(transformed_goal_pose.position.y,transformed_goal_pose.position.x)



                vel = Twist()

                print("angle: ")
                print(angle*180/np.pi)

                if (angle < -self.rad_tol_turn and self._move_state == "turn") or (angle < -self.rad_tol_forvard and self._move_state == "forward"):
                    self._move_state = "turn"
                    print("right")
                    vel.linear.x = 0.0
                    vel.angular.z = np.max([-self.max_turn, self.k_angle*angle - self.m_angle])
                elif (angle > self.rad_tol_turn and self._move_state == "turn") or (angle > self.rad_tol_forvard and self._move_state == "forward"):
                    self._move_state = "turn"
                    print("left")
                    vel.linear.x = 0.0
                    vel.angular.z = np.min([self.max_turn, self.k_angle*angle + self.m_angle])
                elif angle < - self.rad_tol_turn:
                    self._move_state = "forward"
                    print("right and forward")
                    vel.angular.z = np.max([-self.max_turn, self.k_forward_turn*angle - self.m_forward_turn])
                    vel.linear.x = np.min([self.max_speed_forward, self.k_forward*abs(vel.angular.z) + self.m_forward])
                elif angle > self.rad_tol_turn:
                    self._move_state = "forward"
                    print("left and forward")
                    vel.angular.z = np.min([self.max_turn, self.k_forward_turn*angle + self.m_forward_turn])
                    vel.linear.x = np.min([self.max_speed_forward, self.k_forward*abs(vel.angular.z) + self.m_forward])
                else:
                    self._move_state = "forward"
                    print("Foooooooorward")
                    vel.linear.x = 0.3
                    vel.angular.z = 0.0
            elif abs(yaw) > self.yaw_threshold:
                self._move_state = "turn"
                vel = Twist()
                if yaw < -self.yaw_threshold:
                    print("pose fix right")
                    vel.linear.x = 0.0
                    vel.angular.z = np.max([-self.max_turn, self.k_angle*yaw - self.m_angle])
                elif yaw > self.rad_tol_turn:
                    print("pose fix left")
                    vel.linear.x = 0.0
                    vel.angular.z = np.min([self.max_turn, self.k_angle*yaw + self.m_angle])
    
            if vel.linear.x > self._vel_change_threshold + self._current_vel_x or vel.linear.x < self._current_vel_x - self._vel_change_threshold or vel.angular.z > self._vel_change_threshold + self._current_vel_w or vel.angular.z < self._current_vel_w - self._vel_change_threshold:
                dif_x = vel.linear.x -self._current_vel_x
                dif_w = vel.angular.z -self._current_vel_w

                vel.linear.x = self._current_vel_x + self._vel_change_threshold*(dif_x/(abs(dif_x)+abs(dif_w)))
                vel.angular.z = self._current_vel_w + self._vel_change_threshold*(dif_w/(abs(dif_x)+abs(dif_w)))

            if dist < self.slow_down_threshold and stop_at_goal:
                vel.linear.x = np.min(vel.linear.x, self.k_slow_down*dist + self.m_slow_down)
            self._current_vel_x = vel.linear.x
            self._current_vel_w = vel.angular.z
            self._pub_vel.publish(vel)
        if stop_at_goal:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self._pub_vel.publish(vel)
        result = MoveTo.Result()
        result.success = True
        goal_handle.succeed()
        return result
    
    def cancel_callback(self, goal_handle):
        motor_msg = DutyCycles()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.header.frame_id = "motors"

        self._pub_motor.publish(motor_msg)
        return True
    
    def transform(self,goal_pose):
        #wait for tramsform to base_link
        _time = self.get_clock().now().to_msg()

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame="base_link",
            source_frame=goal_pose.header.frame_id,
            time=_time
        )
        
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=2)

        #transforming goal pose to base_link
        try:
            t = self.tf_buffer.lookup_transform(
            "base_link",
            goal_pose.header.frame_id,
            _time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"base_link"} to {goal_pose.header.frame_id}: {ex}')
            return
        print(t)
        return tf2_geometry_msgs.do_transform_pose(goal_pose.pose, t)
        


def main():
    rclpy.init()
    node = Move_to()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()





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