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

class BoxDetectionPublisher(Node):
    def __init__(self):
        super().__init__('box_detection_publisher')
        
        # Create CV bridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
        self.arm_camera_processing = ArmCameraProcessing()
        self.box_detection = BoxDetection()

        # Create callback group for thread safety
        callback_group = ReentrantCallbackGroup()
        
        # Create subscriber for arm camera
        self.camera_subscription = self.create_subscription(
            Image,
            '/arm_camera/filtered/image_raw',
            self.camera_callback,
            10,
            callback_group=callback_group
        )
        
        # Create publisher for box detection images
        self.box_detection_publisher = self.create_publisher(
            Image,
            'box_detection',
            10
        )

        self.box_position_publisher = self.create_publisher(
            Int32MultiArray,
            'box_position',
            10
        )
        
        # Rate limiting to avoid excessive processing
        self.max_rate = 5  # Hz
        self.last_time = self.get_clock().now()
        self.center_points = []
        
        self.get_logger().info('Box detection service initialized')
        self.failed_detections = 0

    def camera_callback(self, msg):
        """Callback function for camera subscriber"""
        # Rate limiting
        current_time = self.get_clock().now()
        if (current_time - self.last_time).nanoseconds / 1e9 < 1.0 / self.max_rate:
            return
        self.last_time = current_time
        
        if self.failed_detections > 10:
            self.center_points = [] 
            msg = Int32MultiArray() 
            msg.data = []
            self.box_position_publisher.publish(msg)
            self.failed_detections = 0

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image to detect boxes
            center_point = self.box_detection.get_contour_from_image(cv_image)
            self.center_points.append(center_point)
            if len(self.center_points) > 10:
                self.center_points.pop(0)

            center_point = np.mean(self.center_points, axis=0)
            cv2.circle(cv_image, (int(center_point[0]), int(center_point[1])), 10, (0, 0, 255), 2)
            processed_image = cv_image
            # Convert processed image back to ROS Image message and publish
            box_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            box_msg.header = msg.header  # Preserve the original header
            self.box_detection_publisher.publish(box_msg)
            msg = Int32MultiArray() 
            msg.data = center_point.astype(int)
            self.box_position_publisher.publish(msg)
            
        except Exception as e:
            # self.get_logger().error(f'Error processing image: {str(e)}')
            msg = Int32MultiArray() 
            msg.data = []
            self.box_position_publisher.publish(msg)
            self.failed_detections += 1

def main(args=None):
    rclpy.init(args=args)
    box_detection_publisher = BoxDetectionPublisher()
    
    try:
        rclpy.spin(box_detection_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        box_detection_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
