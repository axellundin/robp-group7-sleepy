import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from arm_control.arm_camera_processing import ArmCameraProcessing
import numpy as np
from core_interfaces.srv import YoloImageDetect
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class ArmCVTesting(Node):
    def __init__(self):
        super().__init__('arm_cv_testing')
        
        # Create CV bridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
        self.arm_camera_processing = ArmCameraProcessing()
        group1 = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()
        # Create a client to call the object detection service 
        self.yolo_client = self.create_client(YoloImageDetect, 'yolo_image_detect', callback_group=group1)
        while not self.yolo_client.wait_for_service(timeout_sec=10):
            self.get_logger().warning('YOLO service not available, waiting again...')

        # Create subscriber for arm camera
        self.camera_subscription = self.create_subscription(
            Image,
            '/arm_camera/filtered/image_raw',
            self.camera_callback,
            1,
            callback_group=group2
        )
        
        # Create publisher for processed images
        self.grip_publisher = self.create_publisher(
            Image,
            'gripping',
            10
        )
        
        self.max_rate = 5
        self.last_time = self.get_clock().now()
        self.get_logger().info('ArmCVTesting node initialized')

    def camera_callback(self, msg):
        """Callback function for camera subscriber"""
        if (self.get_clock().now() - self.last_time).to_msg().sec < self.max_rate:
            return
        self.last_time = self.get_clock().now()
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Here you can add any image processing you want to do
            # For now, we'll just republish the image
            
            # Call the YOLO service 
            request = YoloImageDetect.Request()
            request.camera_name = "arm_camera"
            request.target_frame = "arm_base_link"
            future = self.yolo_client.call_async(request)

            while not future.done():
                self.get_logger().info('Waiting for YOLO service response...')
                time.sleep(0.5)
            response = future.result()
            self.get_logger().info(f'Got response from YOLO service')
            # If the list of objects is not empty, then we can proceed with the gripping position detection
            if len(response.objects) == 0:
                self.get_logger().info('No objects detected, skipping gripping position detection') 
                return
            # Select an object with category "1.0"
            object_to_grip = None
            prohited = ["toy_white"]
            for obj in response.objects:
                if not obj.category in prohited:
                    if obj.image == Image():
                        continue
                    object_to_grip = obj
                    break
            # If no object with category "1.0" is detected, then we skip gripping position detection
            if object_to_grip is None:
                self.get_logger().info('No object with category "1.0" detected, skipping gripping position detection') 
                return

            # Extract that part of the image 
            image = object_to_grip.image
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

            # Get the edge points 
            edges = self.arm_camera_processing.best_edge_detection(cv_image)
            edge_points = np.where(edges == 255) 
            edge_points = np.array(edge_points).T

            # Get the gripping positions 
            gripping_positions, components = self.arm_camera_processing.get_gripping_position(cv_image)
            
            colors = [tuple(map(int, color)) for color in np.random.randint(0, 255, (len(components), 3))]

            # Create a contour image to publish 
            # Draw the grippable components
            grippable_edges = np.zeros_like(cv_image)
            for i, component in enumerate(components):
                if len(component) > 0:
                    points = edge_points[component]
                    grippable_edges[points[:, 0], points[:, 1]] = colors[i]

            # bg_img = cv_image.copy()
            bg_img = grippable_edges
            # Draw the gripping points
            for i, points in enumerate(gripping_positions):
                cv2.circle(bg_img, (points[0][1], points[0][0]), 5, colors[i], -1)  # First point
                cv2.circle(bg_img, (points[1][1], points[1][0]), 5, colors[i], -1)  # Second point
            
            # Convert back to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(bg_img, encoding='bgr8')
            # ros_image = self.bridge.cv2_to_imgmsg(edges, encoding='8UC1')
            self.grip_publisher.publish(ros_image)
            
        except Exception as e:
            raise e
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ArmCVTesting()
    
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
