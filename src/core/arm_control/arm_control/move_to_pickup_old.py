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
        
        # Joint configurations 
        self.joint_cfgs = [
            {'limits': [0, 24000], 'offset': 0},  # Joint 1
            {'limits': [0, 24000], 'offset': 0},  # Joint 2
            {'limits': [0, 24000], 'offset': 0},  # Joint 3
            {'limits': [0, 24000], 'offset': 0},  # Joint 4
            {'limits': [0, 24000], 'offset': 0},  # Joint 5
        ]
        
        self.get_logger().info('Move To Pickup Action Server has been started')

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
                    'end_effector',
                    'arm_base_link',
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

            current_joint_angles = manipulator_encoders_to_angles(current_joint_encoder_values, self.joint_cfgs)
            end_effector_pose = get_end_effector_pose(current_joint_angles, self.d0, self.d1, self.d2, self.d4)
        
            # Get the trajectory to the object pose
            # Get start pose of the end effector in the arm_base_link frame
            start_pose = np.array([end_effector_pose[0,3],
                                end_effector_pose[1,3],
                                end_effector_pose[2,3]])
            
            # Extract rotation matrix from current end effector pose and convert to quaternion
            start_rotation_matrix = end_effector_pose[0:3, 0:3]
            start_quat = Rotation.from_matrix(start_rotation_matrix).as_quat()
            # start_quat = None
            end_quat = Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()
            # end_quat = None
            
            num_points = 20
            local_pose = np.array([local_pose.position.x, 
                                   local_pose.position.y, 
                                   local_pose.position.z])
            trajectory_pos, trajectory_orient = interpolate_positions(start_pose, local_pose, start_quat, end_quat, num_points)
            # trajectory_pos = interpolate_positions(start_pose, local_pose, start_quat, end_quat, num_points)
            # trajectory_orient = None

            # Move the arm to the object pose
            joint_angles = current_joint_angles
            self.get_logger().info("\n=== Initial State ===")
            self.get_logger().info(f"Joint angles: {np.array2string(np.array(joint_angles), precision=3, suppress_small=True)}")
            
            # Format the pose matrix into position and orientation components
            def format_pose_matrix(pose_matrix):
                pos = pose_matrix[0:3, 3]
                rot = pose_matrix[0:3, 0:3]
                return (f"\nPosition: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
                       f"\nRotation Matrix:\n{np.array2string(rot, precision=3, suppress_small=True)}")
            
            self.get_logger().info(f"End effector pose: {format_pose_matrix(end_effector_pose)}")
            i = 0
            for pos, orient in zip(trajectory_pos, trajectory_orient): 
            # for pos in trajectory_pos: 
                i += 1
                print("-------------------------------- iteration: ", i, " --------------------------------")
                pose = np.eye(4)
                pose[0,3] = pos[0]
                pose[1,3] = pos[1]
                pose[2,3] = pos[2]
                pose[0:3, 0:3] = Rotation.from_quat(orient).as_matrix()
                
                self.get_logger().info("\n=== Target State ===")
                self.get_logger().info(f"Target pose: {format_pose_matrix(pose)}")
                
                end_effector_pose = get_end_effector_pose(joint_angles, self.d0, self.d1, self.d2, self.d4)
                
                self.get_logger().info("\n=== Current State ===")
                self.get_logger().info(f"Current end effector pose: {format_pose_matrix(end_effector_pose)}")
                
                success, joint_angles = inverse_kinematics(joint_angles, pose, self.d0, self.d1, self.d2, self.d4, tolerance=1e-2)
                if not success:
                    self.get_logger().error('Failed to invert kinematics')
                    result.success = False
                    return result

                encoder_values = manipulator_angles_to_encoders(joint_angles, self.joint_cfgs)
                self.get_logger().info(f"Encoder values: {np.array2string(np.array(encoder_values), precision=1)}")
                msg = Int16MultiArray() 
                msg.data = [ 
                    -1,
                    int(encoder_values[4]),
                    int(encoder_values[3]),
                    int(encoder_values[2]),
                    int(encoder_values[1]),
                    int(encoder_values[0]), 
                    200, 
                    200, 
                    200, 
                    200, 
                    200, 
                    200
                ]
                self.multi_servo_cmd_pub.publish(msg)
                time.sleep(0.2)
            # Here you would implement the actual arm movement logic
            # For now, we'll just accept all trajectories as successful
            goal_handle.succeed()
            
        except Exception as e:
            raise e
            print(f"Error: {e}")
            goal_handle.abort()
            
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
