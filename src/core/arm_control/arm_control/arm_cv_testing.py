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
            '/gripping',
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
        category = None
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
            prohited = ["box"]
            for obj in response.objects:
                if not obj.category in prohited:
                    if obj.image == Image():
                        continue
                    object_to_grip = obj
                    category = obj.category
                    break
            # If no object with category "1.0" is detected, then we skip gripping position detection
            if object_to_grip is None:
                self.get_logger().info('No object with category "1.0" detected, skipping gripping position detection') 
                return

            # Extract that part of the image 
            image = object_to_grip.image
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')


            thresh = cv2.Canny(cv_image, 100, 200)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Get the minimum area rectangle   
            contours = [contour for contour in contours if len(contour) > 80]
            cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

            # Convert all contours to convex hull of the union of all contours 
            contours = [cv2.convexHull(np.concatenate(contours)) for contour in contours]

            # Draw contour on image: 
            cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 2)

            # Get the minimum area rectangle    
            if len(contours) == 0:
                self.get_logger().info('No contours detected, skipping gripping position detection') 
                return
            
            max_contour = max(contours, key=cv2.contourArea)
            result = cv2.minAreaRect(max_contour)
            box = cv2.boxPoints(result)
            box = np.intp(box)
            cv2.drawContours(cv_image, [box], 0, (255, 0, 0), 1)

            _, (w,h), angle = result
            min_angle = 0
            max_angle = 0
            if w > h:
                min_angle = angle + 90
                max_angle = angle 
            else:
                min_angle = angle
                max_angle = angle + 90

            min_angle = min_angle * np.pi / 180
            max_angle = max_angle * np.pi / 180
            # Wrap to -pi/2 to pi/2:

            min_angle = np.arctan2(np.sin(min_angle), np.cos(min_angle))
            max_angle = np.arctan2(np.sin(max_angle), np.cos(max_angle))

            min_angle = - ((min_angle + np.pi/2) % np.pi - np.pi/2)
            max_angle = - ((max_angle + np.pi/2) % np.pi - np.pi/2)
            phi = 0

            if category == "cube":
                phi = min(min_angle, max_angle, key=lambda x: abs(x))
            elif category == "toy":
                phi = min_angle
                
            self.get_logger().info(f'min_angle = {min_angle * 180 / np.pi}, max_angle = {max_angle * 180 / np.pi}')
            # phi = np.arctan2(np.sin(phi), np.cos(phi))
            self.get_logger().info(f'category = {category}, phi = {phi * 180 / np.pi}')

            # Convert back to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            #ros_image = self.bridge.cv2_to_imgmsg(thresh, encoding='8UC1')
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
