import rclpy
from rclpy.node import Node
from core_interfaces.srv import YoloImageDetect
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
import cv2 
import os
from cv_bridge import CvBridge  
from sensor_msgs.msg import Image 
import numpy as np

class YoloImageDetectClient(Node):
    def __init__(self):
        super().__init__('yolo_image_detect_client')

        self.client = self.create_client(YoloImageDetect, 'yolo_image_detect')

        while not self.client.wait_for_service(timeout_sec=10):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        request = YoloImageDetect.Request()
        request.camera_name = "rgbd_camera"
        # request.camera_name = "arm_camera"
        request.target_frame = "camera_link"
        # request.target_frame = "arm_base_link"
        # request.target_frame = "camera_depth_optical_frame"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Received response: {len(response.objects)} objects detected.")

            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                category = obj.category
                print(f"Object {i + 1}:")
                print(f"  Category: {category}")
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  Position: {x_m} m, {y_m} m, {z_m} m")

                pose = obj.topleft_point
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  topleft_point: {x_m} m, {y_m} m, {z_m} m")

                pose = obj.bottomright_point
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  bottomright_point: {x_m} m, {y_m} m, {z_m} m")
                bridge=CvBridge()
                if obj.image!=Image():
                    cv_img = bridge.imgmsg_to_cv2(obj.image, desired_encoding='bgr8')
                    cv2.imwrite('./testimage.jpg', cv_img)
                else:
                    pass
        else:
            self.get_logger().error('Failed to receive response.')

class FinalDetectClient(Node):
    def __init__(self):
        super().__init__('final_detect_client')

        self.client = self.create_client(YoloImageDetect, 'final_detect')

        while not self.client.wait_for_service(timeout_sec=10):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        request = YoloImageDetect.Request()
        request.camera_name = "rgbd_camera"
        request.target_frame = "map"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Received response: {len(response.objects)} objects detected.")

            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                category = obj.category
                print(f"Object {i + 1}:")
                print(f"  Category: {category}")
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  Position: {x_m} m, {y_m} m, {z_m} m")

                pose = obj.topleft_point
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  topleft_point: {x_m} m, {y_m} m, {z_m} m")

                pose = obj.bottomright_point
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  bottomright_point: {x_m} m, {y_m} m, {z_m} m")
                bridge=CvBridge()
                if obj.image!=Image():
                    cv_img = bridge.imgmsg_to_cv2(obj.image, desired_encoding='bgr8')
                    cv2.imwrite('./testimage.jpg', cv_img)
                else:
                    pass
        else:
            self.get_logger().error('Failed to receive response.')

class YoloImageSaverClient(Node):
    def __init__(self):
        super().__init__('yolo_image_saver_client')
        self.subscription_arm_image = self.create_subscription(
            Image,
            "/arm_camera/image_raw",
            self.arm_image_listener_callback,
            2
        )
        self.latest_image = None
        self.image_counter = 1
        self.bridge = CvBridge()
        self.save_path = "/home/sleepy/robp-group7-sleepy/yolo/dataset"

    def arm_image_listener_callback(self, msg):
        print("1")
        self.latest_image = msg

    def saveimage(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        image_filename = f"armcamera_image_{self.image_counter:03d}.png"
        image_filepath = os.path.join(self.save_path, image_filename)

        cv2.imwrite(image_filepath, cv_image)
        
        self.get_logger().info(f'Saved image: {image_filepath}')

        self.image_counter += 1



# # this is used to test yolo, and also showing how to use yolo detector node 
# def main():
#     rclpy.init()

#     client = YoloImageDetectClient()

#     try:
#         while rclpy.ok():
#             user_input = input("Press Enter to send a request (Ctrl+C to quit): ")
#             if user_input == '':
#                 debug_time = time.time()
#                 client.send_request()
#                 print(f"Request sent. Time taken: {time.time() - debug_time} seconds.")
#             rclpy.spin_once(client)  # Ensure the node remains active

#     except KeyboardInterrupt:
#         pass  # Handle the interrupt from user (Ctrl+C)

#     client.destroy_node()
#     rclpy.shutdown()


# # this is used to save images from arm camera, in order to generate dataset 
# def main():
#     rclpy.init()

#     client = YoloImageSaverClient()

#     try:
#         while rclpy.ok():
#             user_input = input("Press Enter to send a request (Ctrl+C to quit): ")
#             rclpy.spin_once(client)
#             if user_input == '':
#                 client.saveimage()
#             # rclpy.spin_once(client)  # Ensure the node remains active

#     except KeyboardInterrupt:
#         pass  # Handle the interrupt from user (Ctrl+C)

#     client.destroy_node()
#     rclpy.shutdown()


# this is used to test final detector, node perception mapg
def main():
    rclpy.init()

    client = FinalDetectClient()

    try:
        while rclpy.ok():
            user_input = input("Press Enter to send a request (Ctrl+C to quit): ")
            if user_input == '':
                debug_time = time.time()
                client.send_request()
                print(f"Request sent. Time taken: {time.time() - debug_time} seconds.")
            rclpy.spin_once(client)  # Ensure the node remains active

    except KeyboardInterrupt:
        pass  # Handle the interrupt from user (Ctrl+C)

    client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
