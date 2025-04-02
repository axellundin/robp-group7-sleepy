#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core_interfaces.srv import Place
from std_msgs.msg import Int16MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from core_interfaces.srv import MoveArm
import time
import numpy as np
from rclpy.executors import MultiThreadedExecutor

class PlaceService(Node):
    def __init__(self):
        super().__init__('place_service')
        self.service = self.create_service(
            Place,
            'place',
            self.place_callback
        )
        
        # Define joint positions
        self.joint_movement_time = 1500
        self.gripper_open_position = [3500, -1, -1, -1, -1, -1, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.up_position = [-1, 12000, 12000, 12000, 12000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.place_position = [-1, 12000, 12000, 16000, 8000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        
        self.viewing_position = [-1, 12000, 5000, 21000, 14000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.first_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 7500, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 3000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        
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

    def move_arm_to_joint_values(self, joint_values):
        msg = Int16MultiArray()
        msg.data = joint_values
        self.joint_angles_publisher.publish(msg)
        self.get_logger().info('Moving arm to joint values...')
        latest_command = self.get_clock().now()

        while not self.check_joints(joint_values):
            if self.get_clock().now() - latest_command > rclpy.duration.Duration(seconds=5):
                self.joint_angles_publisher.publish(msg)
                latest_command = self.get_clock().now()
            time.sleep(0.01)
        return True

    def place_callback(self, request, response):
        self.get_logger().info('Starting place callback')

        for viewing_position in [self.viewing_position, self.first_backup_viewing_position, self.backup_viewing_position]:
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
                    response.success = False
                    return response
            else: 
                break
            
        box_pose = self.box_pose
        self.get_logger().info(f'Box position: {box_pose.pose.position}')

        # Check if placement in reachable position
        x, y = box_pose.pose.position.x, box_pose.pose.position.y 
        rho = (x**2 + y**2)**0.5
        if rho > 0.35 or max(x,-y) < 0.15 or y > 0.15: 
            self.get_logger().info('Placement position is not reachable, aborting...')
            response.success = False
            return response 

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
        
        # Open gripper
        self.get_logger().info('Opening gripper...')
        self.move_arm_to_joint_values(self.gripper_open_position)
        time.sleep(2)

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
