#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core_interfaces.srv import Place
from std_msgs.msg import Int16MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from core_interfaces.srv import MoveArm
import time
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from core_interfaces.srv import MoveTo
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class PlaceService(Node):
    def __init__(self):
        super().__init__('place_service')
        self.service = self.create_service(
            Place,
            'place',
            self.place_callback
        )
        
        # Define joint positions
        self.joint_movement_time = 1000
        self.joint_velocity = 10
        self.min_joint_movement_time = 500
        self.gripper_open_position = [3500, -1, -1, -1, -1, -1, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.up_position = [-1, 12000, 12000, 12000, 12000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.place_position = [-1, 12000, 12000, 16000, 8000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        
        self.viewing_position = [-1, 12000, 5000, 21000, 14000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.first_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 7500, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 3000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.second_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 16500, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        
        self.max_joint_diff = 500
        self.max_joint_diff_gripper = 1200
        self.latest_joint_angles = None

        # Publishers and subscribers
        self.joint_angles_publisher = self.create_publisher(
            Int16MultiArray,
            'multi_servo_cmd_sub',
            10
        )

        group = ReentrantCallbackGroup()
        self.joint_angles_subscriber = self.create_subscription(
            JointState,
            'servo_pos_publisher',
            self.joint_angles_callback,
            1,
            callback_group=group
        )
        self.see_box = False 
        self.box_pose = None

        self.box_position_subscriber = self.create_subscription(
            PoseStamped,
            'box_pose',
            self.box_position_callback,
            1,
            callback_group=group
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.move_to_client = self.create_client(MoveTo, 'MoveTo', callback_group=group)
        while not self.move_to_client.wait_for_service(timeout_sec=10):
            self.get_logger().warning('Move to service not available, waiting again...')

        group1 = ReentrantCallbackGroup()
        # Create a client to call the move arm service 
        self.move_arm_client = self.create_client(MoveArm, 'move_arm', callback_group=group1)
        while not self.move_arm_client.wait_for_service(timeout_sec=10):
            self.get_logger().warning('Move arm service not available, waiting again...')

        self.get_logger().info('Place Service Started')

    def joint_angles_callback(self, msg):
        self.latest_joint_angles = msg.position

    def box_position_callback(self, msg):
        pose = msg.pose 
        if pose.position.x == 0 and pose.position.y == 0 and pose.position.z == 0:
            self.see_box = False
            self.box_pose = None
            return 
        else:
            self.see_box = True
            self.box_pose = msg

    def compute_joint_transition_time(self, desired_joint_values):
        if self.latest_joint_angles is None:
            return desired_joint_values
        current_joint_values = self.latest_joint_angles
        speeds = []
        for i in range(6):
            if desired_joint_values[i] == -1:
                speeds.append(self.joint_movement_time)
                continue
            abs_diff = abs(current_joint_values[i] - desired_joint_values[i]) 
            speeds.append(max(int(abs_diff / self.joint_velocity), self.min_joint_movement_time))

        return desired_joint_values[:6] + speeds
    
    def move_arm_to_joint_values(self, joint_values):
        msg = Int16MultiArray()
        msg.data = self.compute_joint_transition_time(joint_values)

        self.joint_angles_publisher.publish(msg)
        self.get_logger().info('Moving arm to joint values...')
        latest_command = self.get_clock().now()

        while not self.check_joints(joint_values):
            if self.get_clock().now() - latest_command > rclpy.duration.Duration(seconds=5):
                msg.data = self.compute_joint_transition_time(joint_values)
                self.joint_angles_publisher.publish(msg)
                latest_command = self.get_clock().now()
            time.sleep(0.01)
        return True

    def check_if_placement_position_is_reachable(self, x, y):
        rho = (x**2 + y**2)**0.5
        if rho > 0.35 or max(x,abs(y)) < 0.15 or y > 0.15: 
            return False
        else:
            return True

    def small_adjustment(self, x, y): 
        self.get_logger().info(f'Starting small adjustment with x={x}, y={y}')
        req_msg = MoveTo.Request()
        goal_list = Path()
        req_msg.threshold = 0.01
        goal_list.header.frame_id = "odom"
        goal_list.header.stamp = self.get_clock().now().to_msg()
    
        # Construct the current pose in the base link frame  
        object_in_odom = self.construct_pose_in_frame(x, y, 0.0, 0.0, "odom", "arm_base_link")
        
        # Get the latest transform from base link to odom 
        odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0)) 
        self.get_logger().info('Retrieved odom transform')

        angle_between_current_pose_and_object = np.arctan2(object_in_odom.pose.position.y - odom_transform.transform.translation.y, object_in_odom.pose.position.x - odom_transform.transform.translation.x)

        place_distance = 0.30 
        arm_base_link_y_shift = 0.0475  
        base_link_to_place_radius = (place_distance**2 + arm_base_link_y_shift**2)**0.5
        base_link_place_angle = np.arctan2(arm_base_link_y_shift, place_distance) 
        target_angle = angle_between_current_pose_and_object - base_link_place_angle  
        target_angle = target_angle
        
        object_pos = np.array([object_in_odom.pose.position.x, object_in_odom.pose.position.y])
        current_pos = np.array([odom_transform.transform.translation.x, odom_transform.transform.translation.y])
        distance_to_object = np.linalg.norm(current_pos - object_pos)
        target_pos = object_pos + (current_pos - object_pos) / distance_to_object * base_link_to_place_radius 

        target_pose = self.construct_pose_in_frame(target_pos[0], target_pos[1], 0.0, target_angle, "odom")

        allow_reverse = False  
        if distance_to_object < base_link_to_place_radius:
            allow_reverse = True 
        self.get_logger().info(f'Target_angle = {target_angle}')
        self.get_logger().info(f'Object pose: {object_in_odom.pose.position.x}, {object_in_odom.pose.position.y}')
        self.get_logger().info(f'Target pose: {target_pose}')

        # Create move to request 
        req_msg = MoveTo.Request()
        req_msg.path.poses = [target_pose]
        req_msg.max_speed = 0.2
        req_msg.allow_reverse = allow_reverse
        req_msg.max_turn_speed = 0.15
        req_msg.enforce_orientation = True 
        req_msg.stop_at_goal = True 

        self.get_logger().info('Sending move_to request...')
        future = self.move_to_client.call_async(req_msg)
        self.get_logger().info('Waiting for move_to response...')
        while not future.done(): 
            time.sleep(0.02) 
            # self.executor.spin_once(timeout_sec=0.1)
        # self.executor.spin_until_future_complete(future)
        result = future.result()
        self.get_logger().info(f'Move_to result: {result}')
        return result.success
    
    def construct_pose_in_frame(self, x, y, z, theta, frame_to, frame_from=None):
        pose = PoseStamped() 
        pose.header.frame_id = frame_to 
        pose.header.stamp = self.get_clock().now().to_msg() 
        pose.pose.position.x = x 
        pose.pose.position.y = y  
        pose.pose.position.z = z  
        pose.pose.orientation.z = np.sin(theta / 2.0)
        pose.pose.orientation.w = np.cos(theta / 2.0)

        if frame_from is not None: 
            transform = self.tf_buffer.lookup_transform(frame_to, frame_from, rclpy.time.Time(seconds=0))
            pose.pose = do_transform_pose(pose.pose, transform)
            pose.header.frame_id = frame_to
        return pose

    def move_to_position(self, box_pose):
        response = MoveArm.Response()
        x, y = box_pose.pose.position.x, box_pose.pose.position.y
        rho = (x**2 + y**2)**0.5
        if rho > 0.35 or max(x,-y) < 0.15 or y > 0.15: 
            return False

        # Compute alpha using formula
        alpha = (1.3 - 0.2) * (rho - 0.14) / (0.35 - 0.14) + 0.2
        phi = np.arctan2(-y, x)
        # Publish the target pose 
        self.get_logger().info('Publishing target pose...')
        request = MoveArm.Request()
        request.pose = box_pose
        request.pose.pose.position.z = -0.02
        request.alpha.data = alpha
        request.phi.data = phi

        future = self.move_arm_client.call_async(request)
        while not future.done():
            self.get_logger().info('Waiting for move arm response...')
            time.sleep(0.1)
        return True 
        
            
    def place_callback(self, request, response):
        self.get_logger().info('Starting place callback')

        while True: 
            for viewing_position in [self.viewing_position, self.first_backup_viewing_position, self.backup_viewing_position, self.second_backup_viewing_position]:
                # Move to place position
                self.get_logger().info('Moving to place position...')
                self.move_arm_to_joint_values(viewing_position)
                time.sleep(2)

                # Get place position from box_position_pub
                for i in range(2): # 2 tries
                    time.sleep(2)
                    self.get_logger().info(f'self.see_box: {self.see_box}')
                    if self.see_box:
                        break 
                    else:
                        self.get_logger().info('No box found, trying again...')
            
                if not self.see_box: 
                    if viewing_position == self.backup_viewing_position:
                        self.get_logger().info('No box found, aborting...')
                        self.move_arm_to_joint_values(self.up_position)
                        response.success = False
                        return response
                else: 
                    break
            
            if self.check_if_placement_position_is_reachable(self.box_pose.pose.position.x, self.box_pose.pose.position.y):
                break
            else:
                self.small_adjustment(self.box_pose.pose.position.x, self.box_pose.pose.position.y)
                self.get_logger().info('Placement position is not reachable, trying again...')
            
        #     # Move to that position and check again. 
        #     box_pose = self.box_pose
        #     self.get_logger().info(f'Box position: {box_pose.pose.position}')

        #     # Check if placement in reachable position
        #     self.move_to_position(box_pose)
            
        #     time.sleep(2)
        #     if self.see_box:
        #         break 
    
        # while True: 
        #     while not self.see_box:
        #             time.sleep(0.1)
        #     if self.check_if_placement_position_is_reachable(self.box_pose.pose.position.x, self.box_pose.pose.position.y):
        #         break
        #     else:
        #         self.move_arm_to_joint_values(self.viewing_position)
        #         while not self.see_box:
        #             time.sleep(0.1)
        #         self.small_adjustment(self.box_pose.pose.position.x, self.box_pose.pose.position.y)
        #         self.get_logger().info('Placement position is not reachable, trying again...')
        #         while not self.see_box:
        #             time.sleep(0.1)

        #         time.sleep(1)

        box_pose = self.box_pose
        self.get_logger().info(f'Box position: {box_pose.pose.position}')

        # Check if placement in reachable position
        if not self.move_to_position(box_pose):
            self.get_logger().info('Placement position is not reachable, aborting...')
            self.move_arm_to_joint_values(self.up_position)
            response.success = False
            return response 
        
        # Open gripper
        self.get_logger().info('Opening gripper...')
        self.move_arm_to_joint_values(self.gripper_open_position)
        time.sleep(0.5)

        # Return to up position
        self.get_logger().info('Moving to up position...')
        self.move_arm_to_joint_values(self.up_position)

        response.success = True
        return response

    def check_joints(self, desired_joint_values):
        desired_joint_values = desired_joint_values[:6]
        indices = [i for i, value in enumerate(desired_joint_values) if value != -1]
        
        if self.latest_joint_angles is not None:
            joint_diff = [abs(self.latest_joint_angles[i] - desired_joint_values[i]) for i in indices]
            max_val = max(joint_diff)
            max_idx = joint_diff.index(max_val)
            if max_idx == 0:
                return max_val < self.max_joint_diff_gripper
            else:
                return max_val < self.max_joint_diff
        return False

def main():
    rclpy.init()
    service = PlaceService()
    
    try:
        ex = MultiThreadedExecutor()
        ex.add_node(service)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
