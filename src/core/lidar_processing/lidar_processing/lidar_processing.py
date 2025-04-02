#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Point
import tf2_geometry_msgs

from _thread import start_new_thread
from core_interfaces.srv import LidarDetect
# from tf2_ros.exceptions import TransformException #comment because error! no module called transformexceotions

class lidar_processing(Node):
    '''
    if the "lidar_launch.yaml" is launched, we will have topic "/scan"
    the node try to listen to /scan topic, read msg, transform it to point cloud 
    and publish(with time stamped and tf frame)
    '''

    def __init__(self):
        super().__init__('lidar_processing')
        self.get_logger().info("lidar inited")

        self.accumulated_points = []
        self.N = 10
        self.count = 0
        self.msg = None

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]  

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_listener_callback,
            10,
        )

        self.srv = self.create_service(LidarDetect, 'lidar_detect', self.lidar_detect_callback)

        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/lidar_pointcloud',  # 话题名称
            10
        )
        print("running queue 50")

    def lidar_listener_callback(self, msg):
        self.msg = msg
        if self.count % self.N == 0:
            self.count+=1
            return 
        self.count = 0 # walk in the process

        # # Get the latest time stamp for a transform between lidar_link and odom frame 
        # odom_transform = self.tf_buffer.lookup_transform(
        #     "odom",
        #     "base_link",
        #     rclpy.time.Time(seconds=0)
        # )
        # # _time = rclpy.time.Time.from_msg(odom_transform.header.stamp)
        # _time = odom_transform.header.stamp

        # Get the transform from lidar_link to odom frame
        # tf_future = self.tf_buffer.wait_for_transform_async(
        #         target_frame="odom",
        #         source_frame="lidar_link",
        #         time=_time
        #     )
        # rclpy.spin_until_future_complete(self, tf_future, timeout_sec=2)
        #transforming goal pose to base_link
        try:
            t = self.tf_buffer.lookup_transform(
            "odom",
            "lidar_link",
            rclpy.time.Time(seconds=0))
        except:
            self.get_logger().info("could not transform from odom to lidar link")
            return

        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        valid_indices = np.isfinite(ranges)# a mask to kick infinity points out
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        x = ranges * np.cos(angles)# make up the points structure
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        points = np.column_stack((x, y, z))

        # Transform the points to odom frame 
        # t.header.stamp = self.get_clock().now().to_msg() 
        t.header.stamp = rclpy.time.Time(seconds=0).to_msg()

        # self.get_logger().info(f"transforming points to odom frame at {t.header.stamp}")
        _, points_transformed = self.transform_points_to_frame(points, t, t.header.stamp)

        pc2_msg = pc2.create_cloud(t.header, self.fields, points_transformed)
        pc2_msg.header.frame_id = "odom"
        
        self.point_cloud_publisher.publish(pc2_msg)

    def lidar_detect_callback(self,request,response):
        print("called")

        frame = request.target_frame
        assert isinstance(frame,str),f"lidar node request should be str, but get {type(frame)} instead"

        self.time_request=self.get_clock().now()
        self.stamp = self.time_request.to_msg()

        _time = rclpy.time.Time.from_msg(self.msg.header.stamp)

        try:
            tf_future = self.tf_buffer.wait_for_transform_async(
                    target_frame=frame,
                    source_frame="lidar_link",
                    time=_time
                )
        except:
            self.get_logger().info(f"could not wait for transform from requested: {frame} to lidarlink")

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(
            frame,
            "lidar_link",
            _time)
        except:
            self.get_logger().info("service could not transform from odom to lidar link")
            return

        angles = np.arange(self.msg.angle_min, self.msg.angle_max, self.msg.angle_increment)
        ranges = np.array(self.msg.ranges)
        valid_indices = np.isfinite(ranges)# a mask to kick infinity points out
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        x = ranges * np.cos(angles)# make up the points structure
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        points = np.column_stack((x, y, z))

        # Transform the points to odom frame 
        points_transformed,_ = self.transform_points_to_frame(points, t, self.msg.header.stamp)
        response = LidarDetect.Response()
        response.lidarpoints = points_transformed

        return response

    def transform_points_to_frame(self, points, transform, stamp):
        """
        transform a group of points together
        """
        transformed_points = []
        transformed_pointslist = []

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            p_stamped = PointStamped()
            p_stamped.header.stamp = stamp
            p_stamped.header.frame_id = "lidar_link" 
            p_stamped.point = p  
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
            newp = Point()
            newp.x = transformed_point_stamped.point.x
            newp.y = transformed_point_stamped.point.y
            newp.z = transformed_point_stamped.point.z
            transformed_points.append(newp)
            transformed_pointslist.append([transformed_point_stamped.point.x,transformed_point_stamped.point.y,transformed_point_stamped.point.z])
        return transformed_points, transformed_pointslist   

def main():
    rclpy.init()
    node = lidar_processing()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()