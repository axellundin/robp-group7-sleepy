import torch
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
import time
from cv_bridge import CvBridge  
import cv2  

time_it_enable = True
trigger_times = 1000

def time_it(func):
    """ 
    calculate the time used for executing the function
    """
    def wrapper(*args, **kwargs):
        if time_it_enable:  
            start_time = time.time() 
            result = func(*args, **kwargs) 
            end_time = time.time()  
            elapsed_time = end_time - start_time 
            print(f"Function '{func.__name__}' executed in {elapsed_time:.4f} seconds")
        else:
            result = func(*args, **kwargs)  
        return result  
    return wrapper

class YoloDetector(Node):
    @time_it
    def __init__(self):
        super().__init__('topic_listener_node')

        self.model = YOLO("/home/sleepy/robp-group7-sleepy/yolo/models/best.pt")

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            # '/camera/camera/color/image_raw',
            "/arm_camera/image_raw",
            self.listener_callback,
            10
        )

        self.receiving_message = False  
        self.triggering = False

    @time_it
    def timer_callback(self):
        global trigger_times
        if not self.triggering and trigger_times>0:
            trigger_times-=1
            self.get_logger().info('Timer triggered, enabling listener callback.')
            self.triggering = True 


    @time_it
    def listener_callback(self, msg):
        if self.triggering:
            self.get_logger().info(f'Received message')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.model(cv_image,
                            )
            result[0].save(filename="result.jpg")
            result[0].show()

            self.triggering = False

def main():
    rclpy.init()

    node = YoloDetector()

    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


    

if __name__ == "__main__":
    main()


