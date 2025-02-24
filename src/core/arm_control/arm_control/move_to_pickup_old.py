#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from core_interfaces.action import MoveToPickup
from arm_control.arm_utils import *
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from sensor_msgs.msg import JointState
from arm_control.interpolate import *
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16MultiArray
import time
from arm_control.kinematics import Kinematics
class MoveArmActionServer(Node):
    def __init__(self):
        super().__init__('move_to_pickup_action_server')
        
        # Create a reentrant callback group to allow multiple callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Create a publisher to /multi_servo_cmd_sub 
        self.multi_servo_cmd_pub = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            10
        )

        # Create the action server
        self._action_server = ActionServer(
            self,
            MoveToPickup,
            'move_to_pickup',
            self.execute_callback,
            callback_group=self.callback_group
        )

         # DH parameters 
        self.d0 = 0
        self.d1 = 0.001 * 101 # mm -> m
        self.d2 = 0.001 * 95 
        self.d3 = 0 
        self.d4 = 0.001 * 168 

        self.joint_limits = np.array([
            [-np.pi * 120 / 180, np.pi * 120 / 180], 
            [-np.pi * 30 / 180, np.pi * 90 / 180], 
            [-np.pi * 60 / 180, np.pi * 120 / 180], 
            [-np.pi * 60 / 180, np.pi * 120 / 180], 
            [-np.pi * 120 / 180, np.pi * 120 / 180]
        ])

        self.encoder_limits = np.array([
            [0, 24000],
            [6000, 21000],
            [3000, 24000],
            [3000, 24000],
            [0, 24000]
        ])
        self.kinematics = Kinematics(self.joint_limits)
        self.get_logger().info('Move To Pickup Action Server has been started')

    def rotation_matrix(self, axis, angle):
        """
        Returns 4x4 rotation matrix for a given axis and angle
        """
        if axis == 'x':
            return np.array([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
        elif axis == 'y':
            return np.array([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
        elif axis == 'z':
            return np.array([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    async def wait_for_message(self, topic_name, msg_type, callback_group=None):
        """Wait for a message on the specified topic."""
        future = rclpy.task.Future()
        
        def callback(msg):
            nonlocal future
            future.set_result(msg)
            sub.destroy()
            
        sub = self.create_subscription(
            msg_type,
            topic_name,
            callback,
            1,
            callback_group=callback_group
        )
        
        try:
            return await future
        except Exception as e:
            self.get_logger().error(f'Error waiting for message: {str(e)}')
            return None

    async def get_single_message(self, topic, msg_type):
        """Get a single message from a topic."""
        future = rclpy.task.Future()
        subscription = None
        
        def callback(msg):
            nonlocal future
            if not future.done():
                future.set_result(msg)
        
        subscription = self.create_subscription(
            msg_type,
            topic,
            callback,
            1,
            callback_group=self.callback_group
        )
        
        try:
            result = await future
            self.destroy_subscription(subscription)  # Move destruction after getting result
            return result
        except Exception as e:
            self.get_logger().error(f'Failed to get message: {str(e)}')
            if subscription:
                self.destroy_subscription(subscription)
            return None
        
    def inverse_kinematics(self, joint_angles, target_pose): 
        success, joint_angles = self.kinematics.inverse_kinematics(joint_angles, target_pose, tolerance=1e-2)
        return success, joint_angles


    def check_encoder_limits(self, joint_encoder_values):
        for i in range(len(joint_encoder_values)):
            if joint_encoder_values[i] < self.encoder_limits[i][0] or joint_encoder_values[i] > self.encoder_limits[i][1]:
                return False
        return True
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        try:
            
            # Create result message
            result = MoveToPickup.Result()

            # Transform the object pose to the arm_base_link frame
            object_pose = goal_handle.request.object_pose
            object_pose.header.frame_id = 'arm_base_link'
            object_pose.header.stamp = rclpy.time.Time(seconds=0).to_msg()

            # Get the transform from the object frame to the arm_base_link frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'arm_base_link',
                    object_pose.header.frame_id,
                    rclpy.time.Time(seconds=0)
                )
            except TransformException as e:
                self.get_logger().error(f'Transform error: {str(e)}')
                result.success = False
                return result
            
            # Convert the object pose to the arm_base_link frame
            local_pose = tf2_geometry_msgs.do_transform_pose(object_pose.pose, transform)
            
            # Get the transform from arm_base_link to end_effector frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'arm_base_link',
                    'end_effector',
                    rclpy.time.Time(seconds=0)
                )
            except TransformException as e:
                self.get_logger().error(f'Transform error: {str(e)}')
                result.success = False
                return result
            

            # Get current joint angles 
            joint_values_msg = await self.get_single_message(
                'servo_pos_publisher',
                JointState
            )
            
            if joint_values_msg is None:
                self.get_logger().error('No joint values message received')
                result.success = False
                return result

            current_joint_encoder_values = joint_values_msg.position[1:][::-1]

            current_joint_angles = self.kinematics.manipulator_encoders_to_angles(current_joint_encoder_values)
            # end_effector_pose = get_end_effector_pose(current_joint_angles, self.d0, self.d1, self.d2, self.d4)
            # end_effector_pose = self.kinematics.fw_kinematics(current_joint_angles)
            print("current_joint_angles: ", current_joint_angles)
            end_effector_pose = self.kinematics.fw_kinematics(current_joint_angles)
            goal = np.array([local_pose.position.x, local_pose.position.y, local_pose.position.z])
            goal_as_matrix = np.eye(4) 
            goal_as_matrix[:3, 3] = goal 
            orientation = self.rotation_matrix('y', np.pi/2)
            # orientation = self.rotation_matrix('x', np.pi/4) @ orientation
            goal_as_matrix = goal_as_matrix @ orientation
            print(f"Start: {end_effector_pose}")
            print(f"Transform: {transform}")
            print(f"Goal: {goal}")

            time.sleep(10)

            success, goal_angles = self.inverse_kinematics(current_joint_angles, goal_as_matrix)
            
            if not success:
                self.get_logger().error('Failed to generate trajectory')
                result.success = False
                goal_handle.abort()
                return result
            
            encoder_values = self.kinematics.manipulator_angles_to_encoders(goal_angles)
            self.get_logger().info(f"Encoder values: {np.array2string(np.array(encoder_values), precision=1)}")
            msg = Int16MultiArray() 
            if not self.check_encoder_limits(encoder_values):
                self.get_logger().error('Encoder values out of limits')
                result.success = False
                goal_handle.abort()
                return result

            msg.data = [ 
                -1,
                int(encoder_values[4]),
                int(encoder_values[3]),
                int(encoder_values[2]),
                int(encoder_values[1]),
                int(encoder_values[0]), 
                2000, 
                2000, 
                2000, 
                2000, 
                2000, 
                2000
            ]
            self.get_logger().info(f"I want to publish encoder values: {np.array2string(np.array(msg.data), precision=1)}")
            self.multi_servo_cmd_pub.publish(msg)
            # Here you would implement the actual arm movement logic
            # For now, we'll just accept all trajectories as successful
            goal_handle.succeed()
            # result = True 
            
        except Exception as e:
            print(f"Error: {e}")
            goal_handle.abort()
            raise e
            
        return result

def main(args=None):
    rclpy.init(args=args)
    move_arm_action_server = MoveArmActionServer()
    
    try:
        rclpy.spin(move_arm_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        move_arm_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
