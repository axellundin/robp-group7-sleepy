import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
from arm_control.arm_utils import (
    manipulator_encoders_to_angles,
    get_joint_position,
)
import numpy as np

class ArmJointTFPublisher(Node):
    def __init__(self):
        super().__init__('arm_joint_tf_publisher')
        
        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribe to servo positions
        print("Subscribing to servo positions")
        self.subscription = self.create_subscription(
            JointState,
            'servo_pos_publisher',
            self.servo_callback,
            10
        )
        print("Subscribed to servo positions")
        
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
        
        # Frame names
        self.frame_names = [
            # 'joint0',
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'end_effector'
        ]

    def servo_callback(self, msg):
        # Convert encoder values to angles
        print("Received servo positions")
        print(msg.position)
        joint_angles = manipulator_encoders_to_angles(msg.position[::-1], self.joint_cfgs)

        # Publish TF for each joint
        for i in range(len(self.frame_names)):
            # Get transformation matrix for current joint
            transform = get_joint_position(
                joint_angles, 
                self.d0, 
                self.d1, 
                self.d2, 
                self.d4, 
                i + 1
            )
            
            # Create and fill TransformStamped message
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'arm_base_link'
            t.child_frame_id = self.frame_names[i]
            
            # Extract position from transformation matrix
            t.transform.translation.x = transform[0, 3]
            t.transform.translation.y = transform[1, 3]
            t.transform.translation.z = transform[2, 3]
            
            # Convert rotation matrix to quaternion
            R = transform[:3, :3]
            trace = np.trace(R)
            
            if trace > 0:
                S = np.sqrt(trace + 1.0) * 2
                t.transform.rotation.w = 0.25 * S
                t.transform.rotation.x = (R[2, 1] - R[1, 2]) / S
                t.transform.rotation.y = (R[0, 2] - R[2, 0]) / S
                t.transform.rotation.z = (R[1, 0] - R[0, 1]) / S
            else:
                if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    t.transform.rotation.w = (R[2, 1] - R[1, 2]) / S
                    t.transform.rotation.x = 0.25 * S
                    t.transform.rotation.y = (R[0, 1] + R[1, 0]) / S
                    t.transform.rotation.z = (R[0, 2] + R[2, 0]) / S
                elif R[1, 1] > R[2, 2]:
                    S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    t.transform.rotation.w = (R[0, 2] - R[2, 0]) / S
                    t.transform.rotation.x = (R[0, 1] + R[1, 0]) / S
                    t.transform.rotation.y = 0.25 * S
                    t.transform.rotation.z = (R[1, 2] + R[2, 1]) / S
                else:
                    S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    t.transform.rotation.w = (R[1, 0] - R[0, 1]) / S
                    t.transform.rotation.x = (R[0, 2] + R[2, 0]) / S
                    t.transform.rotation.y = (R[1, 2] + R[2, 1]) / S
                    t.transform.rotation.z = 0.25 * S
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArmJointTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
