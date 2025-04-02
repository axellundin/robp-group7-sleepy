import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from core_interfaces.msg import PointcloudDetectedObj
from core_interfaces.srv import PointcloudDetect
import time

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(PointcloudDetect, 'pointcloud_detect')  # Service name
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service available!')

    def send_request(self):
        # Create a request
        request = PointcloudDetect.Request()
        request.target_frame = str("camera_depth_optical_frame")

        # Call the service
        self.debug_time=time.time()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response with {len(response.objects)} objects")
            for obj in response.objects:
                self.get_logger().info(f"item: class {obj.category} position:{obj.pose.pose.position.x},{obj.pose.pose.position.y},{obj.pose.pose.position.z}")  # Modify based on the actual object message structure
                self.get_logger().info(f"               orientation:{obj.pose.pose.orientation.x},{obj.pose.pose.orientation.y},{obj.pose.pose.orientation.z},{obj.pose.pose.orientation.w}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        print(f"time spent {time.time()-self.debug_time}")


def main(args=None):
    rclpy.init(args=args)
    client = ServiceClient()

    try:
        while rclpy.ok():
            input("Press Enter to send request...")
            client.send_request()
            rclpy.spin_once(client)  # Allow time for the service to process
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
