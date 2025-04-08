import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from .map import MapNode as Map
import pdb
import traceback
import time 

from core_interfaces.srv import LidarDetect
from core_interfaces.srv import PointcloudDetect
from core_interfaces.srv import YoloImageDetect
from core_interfaces.srv import LidarlikeFromCamera
from core_interfaces.srv import CameraDetect
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster, TransformStamped


class MapGenerator(Node):
    def __init__(self):
        super().__init__('my_ros2_node')
        
        self.timer = None  
        # self.timer = self.create_timer(5, self.timer_callback) 

        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            Bool,
            'stimulation_for_grid_map',  
            self.callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.srv = self.create_service(YoloImageDetect, 'final_detect', self.final_detect_callback)

        self.lidarclient = self.create_client(LidarDetect, 'lidar_detect',callback_group=self.callback_group)
        self.lidarclient.wait_for_service(timeout_sec=1)
        self.pointcloudclient = self.create_client(PointcloudDetect, 'pointcloud_detect',callback_group=self.callback_group)
        self.pointcloudclient.wait_for_service(timeout_sec=1)
        self.yoloimageclient = self.create_client(YoloImageDetect, 'yolo_image_detect',callback_group=self.callback_group)
        self.yoloimageclient.wait_for_service(timeout_sec=1)
        self.lidarfromcamclient = self.create_client(LidarlikeFromCamera, 'lidarlike_from_camera',callback_group=self.callback_group)
        self.lidarfromcamclient.wait_for_service(timeout_sec=1)
        self.cameraclient = self.create_client(CameraDetect, "camera_detect", callback_group=self.callback_group)
        self.cameraclient.wait_for_service(timeout_sec=1)

        self.map = Map()
        self.subscription 

    def final_detect_callback(self,request,response):
        def publish_transform(child_frame_id, pose, father_frame_id = 'map'):
            transform = TransformStamped()

            # transform.header.stamp = self.get_clock().now()

            transform.header.frame_id = father_frame_id
            transform.child_frame_id = child_frame_id

            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z

            transform.transform.rotation.x = pose.pose.orientation.x
            transform.transform.rotation.y = pose.pose.orientation.y
            transform.transform.rotation.z = pose.pose.orientation.z
            transform.transform.rotation.w = pose.pose.orientation.w

            # 发布变换
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().info(f"Dynamic transform published: {child_frame_id}")
        assert request.camera_name == "rgbd_camera", "camera_name must be rgbd camera"
        assert request.target_frame == "map", "target_frame must be map"

        process_result = self.process_once_s()
        for index, i in enumerate(process_result):
            name = (f"{i.category}_num{index}")
            publish_transform(name, i.center_point)

        response.objects = process_result
        response.category_list = "aha look at my face"
        
        return response

    def callback(self, msg):
        assert isinstance(msg.data, bool), f"Expected map generator stimulation to be bool, but got {type(msg.data)}"

        if msg.data:
            self.get_logger().info("Received True, calling function immediately.")
            self.process_once()
            if self.timer:
                self.timer.cancel()
                self.timer = None
        else:
            self.get_logger().info("Received False, calling function periodically.")
            if self.timer is None:
                self.timer = self.create_timer(7, self.timer_callback) 

    def timer_callback(self):
        try:
            # self.process_once()
            self.process_once_v2()

        except Exception as e:
            error_message = traceback.format_exc()
            self.get_logger().error(f"Error occurred in MapGenerator: {error_message}")

    def process_once(self):
        print("process once-----------------------------------")
        start_time = time.time()

        self.map.re_init_cache_map()
        self.map.get_robot_pose()
        self.map.add_explored_area((50,50,0))

        lidarpoints = self.request_lidar_srv()
        self.map.add_lidar_result(lidarpoints)
        # self.get_logger().info("done lidar")

        pc_detected_objs = self.request_pointcloud_srv()
        self.map.add_pointcloud_result(pc_detected_objs)
        # self.get_logger().info("done pc")

        yolo_detected_objs = self.request_yolo_srv()
        self.map.add_yolo_result(yolo_detected_objs)
        # self.get_logger().info("done yolo")

        self.map.update_map_with_intention()
        self.map.publish_map(self.map.grid_map)

        print("time consumption:")
        print(time.time() - start_time)

    def process_once_v2(self):
        print("process once v2-----------------------------------")
        start_time = time.time()

        self.map.re_init()
        self.map.get_robot_pose()

        # self.map.add_explored_area()
        # lidarpoints = self.request_lidar_srv()
        # self.map.add_lidar_result(lidarpoints)
        # print("lidar time consumption:")
        # print(time.time() - start_time)  
        # start_time = time.time() 

        camera_detect_result = self.request_camera_srv()
        self.map.add_lidarfromcam_result(camera_detect_result.points)
        self.map.add_pointcloud_result_v2(camera_detect_result.objects)

        print("camera detect time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        yolo_detected_objs = self.request_yolo_srv()
        self.map.add_yolo_result_v2(yolo_detected_objs)

        print("yolo time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        self.map.key_strategy()

        self.map.update_map_with_intention_v2()

        self.map.get_logger().info(str(self.map.mappoint_dictlist))
        # self.map.publish_map(self.map.grid_map)

        print("strategy time consumption:")
        print(time.time() - start_time)       

    def process_once_s(self):
        start_time = time.time()

        self.map.re_init()
        camera_detect_result = self.request_camera_srv()
        self.map.add_pointcloud_result_v2(camera_detect_result.objects)

        print("camera detect time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        yolo_detected_objs = self.request_yolo_srv()
        self.map.add_yolo_result_v2(yolo_detected_objs)

        print("yolo detect time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        result = self.map.key_strategy_lite()

        print("strategy time consumption:")
        print(time.time() - start_time)  

        return result

    def request_lidar_srv(self):
        request = LidarDetect.Request()
        request.target_frame = "map"
        future = self.lidarclient.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1)

        if future.result() is not None:
            # self.get_logger().info(f"Received lidar points: {future.result().lidarpoints}")
            self.get_logger().info("Received lidar points")
            return  future.result().lidarpoints
        
        else:
            self.get_logger().error('Service call failed.')

    def request_pointcloud_srv(self):
        request = PointcloudDetect.Request()
        request.target_frame = "map"
        future = self.pointcloudclient.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1)

        if future.result() is not None:
            self.get_logger().info(f"Received rgbd pointcloud detected objs")
            return  future.result().objects# cagetory item or box pose(s) 
        else:
            self.get_logger().error('Service call failed.')

    def request_yolo_srv(self):
        request = YoloImageDetect.Request()
        request.target_frame = "map"
        request.camera_name = "rgbd_camera"
        future = self.yoloimageclient.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        
        if future.result() is not None:
            self.get_logger().info("Received yolo detected objs")
            # self.get_logger().info(future.result().objects)
            return  future.result()
        else:
            self.get_logger().error('Service call failed.')

    def request_lidarfromcam_srv(self):
        request = LidarlikeFromCamera.Request()
        request.height = float(0)
        request.target_frame = "map"
        future = self.lidarfromcamclient.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1)
        
        if future.result() is not None:
            self.get_logger().info("Received lidarfromcam points")
            return  future.result().points 
        else:
            self.get_logger().error('Service call failed.')

    def request_camera_srv(self):
        request = CameraDetect.Request()
        request.height = float(0.03)
        request.target_frame = "map"
        future = self.cameraclient.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1)
        
        if future.result() is not None:
            self.get_logger().info("Received camera detect result")
            return  future.result() 
        else:
            self.get_logger().error('Service call failed.')


def main():
    rclpy.init()
    node = MapGenerator()
    # ex = MultiThreadedExecutor()
    # ex.add_node(node)

    try:
        # ex.spin()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
