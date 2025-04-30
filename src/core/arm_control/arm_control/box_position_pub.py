import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from arm_control.arm_camera_processing import ArmCameraProcessing
from arm_control.arm_camera_processing import BoxDetection
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Int32MultiArray
from arm_control.arm_camera_processing import BoxPositionConverter
from geometry_msgs.msg import PoseStamped

def quaternion_to_matrix(quaternion):
    """
    Convert a quaternion to a transformation matrix.
    
    Args:
        quaternion: [x, y, z, w] quaternion
        
    Returns:
        4x4 transformation matrix (numpy array)
    """
    x, y, z, w = quaternion
    
    # Initialize the rotation matrix
    matrix = np.zeros((4, 4), dtype=np.float64)
    
    # Fill in rotation components
    xx, xy, xz = x*x, x*y, x*z
    yy, yz, zz = y*y, y*z, z*z
    wx, wy, wz = w*x, w*y, w*z
    
    matrix[0, 0] = 1.0 - 2.0 * (yy + zz)
    matrix[0, 1] = 2.0 * (xy - wz)
    matrix[0, 2] = 2.0 * (xz + wy)
    
    matrix[1, 0] = 2.0 * (xy + wz)
    matrix[1, 1] = 1.0 - 2.0 * (xx + zz)
    matrix[1, 2] = 2.0 * (yz - wx)
    
    matrix[2, 0] = 2.0 * (xz - wy)
    matrix[2, 1] = 2.0 * (yz + wx)
    matrix[2, 2] = 1.0 - 2.0 * (xx + yy)
    
    # Set the bottom row for homogeneous coordinates
    matrix[3, 3] = 1.0
    
    return matrix

class BoxPositionPublisher(Node):
    def __init__(self):
        super().__init__('box_position_publisher')
        
        # Create CV bridge for converting between ROS and OpenCV images
        self.box_position_converter = BoxPositionConverter()
        # Create callback group for thread safety
        callback_group = ReentrantCallbackGroup()
        
        # Create subscriber for arm camera
        self.camera_subscription = self.create_subscription(
            Int32MultiArray,
            'box_position',
            self.box_position_callback,
            10,
            callback_group=callback_group
        )

        self.box_position_publisher = self.create_publisher(
            PoseStamped,
            'box_pose',
            10
        )

        # TF listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Box position publisher initialized')

    def box_position_callback(self, msg):
        """Callback function for box position subscriber"""
        # Rate limiting
        data = msg.data 
        if len( data ) == 0: 
            # Publish "no box"
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'arm_base_link'
            self.box_position_publisher.publish(pose)
            return 

        # Convert to world coordinates
        x, y = data[0], data[1]
        # Try to get transform from arm_camera_link to arm_base_link
        try:
            transform = self.tf_buffer.lookup_transform('arm_base_link', 'arm_camera', rclpy.time.Time())
        except Exception as ex:
            self.get_logger().error('Could not transform from arm_camera_link to arm_base_link: %s' % ex)
            return 
        
        # Get matrix of whole transform
        # Extract translation and rotation from transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        # Convert quaternion to rotation matrix
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        transform_matrix = quaternion_to_matrix(quat)
        
        # Add translation to transformation matrix
        transform_matrix[0, 3] = translation.x
        transform_matrix[1, 3] = translation.y
        transform_matrix[2, 3] = translation.z

        world_position = self.box_position_converter.convert_image_to_world_coordinates(transform_matrix, x, y, -0.065)
        print(f"World position: {world_position}")
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'arm_base_link'
        pose.pose.position.x = world_position[0]
        pose.pose.position.y = world_position[1]
        pose.pose.position.z = world_position[2]
        self.box_position_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    box_position_publisher = BoxPositionPublisher()
    
    try:
        rclpy.spin(box_position_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        box_position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
