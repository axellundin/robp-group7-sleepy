#!/usr/bin/env python

import math
from math import atan2, sqrt
import time
import numpy as np

import rclpy
from rclpy.node import Node

from collections import Counter

from sensor_msgs.msg import PointCloud2
import std_msgs
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import Pose, do_transform_pose
from .ransac import RANSACPlaneDetector as RSPD
from .DBSCAN import better_DBSCAN
from geometry_msgs.msg import PoseStamped

import ctypes
import struct

from core_interfaces.msg import DetectedMsg

time_it_enable = True

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

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        print("started")

        self.detected_objects_publisher = self.create_publisher(
            DetectedMsg,
            'detected',
            10)

        self.detection_max_distance = 0.9
        self.time_it_times = 1

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a transform listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the publisher
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/camera/camera/depth/color/ds_pointcloud', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)
        
        self.create_subscription(
            PointCloud2, '/lidar_pointcloud', self.lidar_callback, 10)

    def lidar_callback(self, msg: PointCloud2):
        self.lidar_pointcloud = msg

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.
        """
        global time_it_enable
        if self.time_it_times<=0:
            time_it_enable = False

        self.objects = []
        self.boxes = [] 

        # here start the show-----------------------------------------------------------------

        # faster pc to munpy
        points, colors = self.pointcloud_to_points_and_colors_optimized(msg)

       # faster downsample
        points, colors = self.voxel_grid_downsample_optimized(points,colors, 0.01)

        #print some shit
        # self.debug_points(points)

        #filter points for ground
        points, colors = self.filter_points(points, colors, 0.08)

        #create pc for publishing 
        pointcloud2 = create_pointcloud2(points, colors, "camera_depth_optical_frame", msg.header.stamp)

        self.pointcloud_pub.publish(pointcloud2)

        cluster = self.cluster_points(points, colors)
        self.deal_with_clustered_points(cluster)


        msg = DetectedMsg() 
        msg.objects = self.objects
        msg.boxes = self.boxes
        print("published once")
        print(msg)
        self.detected_objects_publisher.publish(msg)
        print("finish one round")

        self.time_it_times -= 1

    @time_it
    def pointcloud_to_points_and_colors(self, msg):
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        remove_indicies = []
        for idx, x in enumerate(gen):
            if np.linalg.norm(x[:3]) > self.detection_max_distance:
                remove_indicies.append(idx)
                continue
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)
        
        for idx, val in enumerate(remove_indicies):# just to make sure remove indicies is int
            if not isinstance(val, int):  
                print(f"Index {idx} is not an integer: {val}")       
        if len(remove_indicies) == 0:
            print("No indices to remove.")
        else:
            remove_indicies = np.array(remove_indicies).astype(int)    


        points = np.delete(points, np.array(remove_indicies), axis=0)
        colors = np.delete(colors, np.array(remove_indicies), axis=0)
        colors = colors.astype(np.float32) / 255
        return points, colors

    @time_it
    def pointcloud_to_points_and_colors_optimized(self, msg):
        """ fast speed from pointcloud to points and colors """
        
        def convert_color_to_rgb(colors):
            rgb_colors = np.empty((colors.shape[0], 3), dtype=np.uint8)
            for idx, c in enumerate(colors):
                s = struct.pack('>f', c)  
                i = struct.unpack('>l', s)[0] 
                pack = ctypes.c_uint32(i).value
                rgb_colors[idx, 0] = (pack >> 16) & 255
                rgb_colors[idx, 1] = (pack >> 8) & 255
                rgb_colors[idx, 2] = pack & 255
            return rgb_colors
        
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]  
        colors = gen[:, 3]  

        distances = np.linalg.norm(points, axis=1)
        valid_mask = distances <= self.detection_max_distance 

        points = points[valid_mask]
        colors = colors[valid_mask]

        colors = convert_color_to_rgb(colors)
        colors = colors.astype(np.float32) / 255.0

        return points, colors

    @time_it
    def filter_points(self, points, colors, min_height = 0.1):
        filtered_points = points[points[:, 1] <= min_height]
        filtered_colors = colors[points[:, 1] <= min_height]
        return filtered_points, filtered_colors

    @time_it
    def cluster_points(self, points, colors, eps=0.02, min_samples=6):
        """
        Use DBSCAN (Density-Based Spatial Clustering of Applications with Noise) to group points into clusters.
        Points that are in a dense region will be grouped together as a cluster. Points that do not belong to any cluster will be marked as noise.

        :param points: (numpy.ndarray) The input points to be clustered.
        :param colors: (numpy.ndarray) The colors corresponding to each point.
        :param eps: (float, optional) The maximum distance between two points for one to be considered as in the neighborhood of the other. Default is 0.02.
        :param min_samples: (int, optional) The number of points required to form a dense region (a cluster). Default is 6.

        Returns:
        - sorted_clusters (dict): A dictionary where:
            - Key: cluster ID (int), which is a unique identifier for each cluster.
            - Value: a dictionary containing two keys:
                - 'points' (numpy.ndarray): The coordinates of the points in the cluster.
                - 'colors' (numpy.ndarray): The colors corresponding to the points in the cluster.
        """
        db = better_DBSCAN(eps=eps, min_samples=min_samples)  
        sorted_clusters = db.fit_predict(points, colors)

        # open if you want to debug
        for cluster_id, data in sorted_clusters.items():
            print(f"Cluster {cluster_id} has {len(data['points'])} points.")
        
        return sorted_clusters

    @time_it
    def deal_with_clustered_points(self, cluster):
        
        if not cluster:
            self.get_logger().info("No clusters found. Returning without processing.")
            return
        
        for cluster_id, cluster_data in cluster.items():
            points = cluster_data['points']
            colors = cluster_data['colors']

            avg_position = np.mean(points, axis=0)
            avg_color = np.mean(colors, axis=0)
        
            if len(points) > 120:
                # pose = Pose()
                # pose.position.x = avg_position[0]
                # pose.position.y = avg_position[1]
                # pose.position.z = avg_position[2]                
                # self.publish_transform(f"box_{cluster_id}", pose)

                detector = RSPD()
                planes, inliers, remained_points = detector.ransac_plane_detection(
                    points, 3, 0.006, max_trials=200, stop_inliers_ratio=0.9, 
                    out_layer_inliers_threshold=80, out_layer_remains_threshold=30
                )

                # planes cx cy cz nx ny nz  
                # cx is right, cy is down, cz is far from camera 
                # nz is biggest vertical to camera

                if len(planes) == 2:
                    center_list = []
                    normal_list = []
                    for i, plane in enumerate(planes):
                        # print(f"平面 {i+1} 的参数：{plane}")
                        # print(f"平面 {i+1} 的内点数量：{inliers[i].shape[0]}")
                        pose, center, normal = self.plane_to_pose(plane)
                        center_list.append(center)
                        normal_list.append(normal)
                        # self.publish_transform(f"box_face_{i}", pose)
                    pose = self.calc_plane_center(center_list[0],normal_list[0],center_list[1],normal_list[1])
                    self.publish_transform(f"box_center",pose)
                    self.boxes.append(pose)
                else:
                    for i, plane in enumerate(planes):
                        pose, center, normal = self.plane_to_pose(plane)
                        self.publish_transform(f"box_{i}",pose)
                        self.boxes.append(pose)                   

            else:                
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "camera_depth_optical_frame"
                pose.pose.position.x = avg_position[0]
                pose.pose.position.y = avg_position[1]
                pose.pose.position.z = avg_position[2]
                self.publish_transform(f"obj_{cluster_id}", pose)
                self.objects.append(pose)

    def calc_plane_center(self, center1,normal1,center2,normal2):
        # we just take 0 as x and 2(z) as y and the real y is fixed
        A1, B1 = normal1[2], -normal1[0]
        C1 = A1*center1[0] + B1*center1[2]
        A2, B2 = normal2[2], -normal2[0]
        C2 = A2*center2[0] + B2*center2[2]

        A = np.array([[A1, B1], [A2, B2]]) 
        B = np.array([C1, C2])  

        intersection = np.linalg.solve(A, B)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_optical_frame"
        pose.pose.position.x = intersection[0]
        pose.pose.position.y = center1[1]
        pose.pose.position.z = intersection[1]

        return pose

    def plane_to_pose(self,plane_params):
        center = plane_params[:3] 
        normal = plane_params[3:]  

        center[1] = 0.04

        if normal[2] > 0:  
            normal = -normal  

        roll = 0.0
        pitch = -atan2(normal[2], normal[0]) 
        print(pitch)
        yaw = 0.0  
        
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_optical_frame"
        pose.pose.position.x = center[0]
        pose.pose.position.y = center[1]
        pose.pose.position.z = center[2]
        
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        return pose, center, normal
    
    @time_it
    def voxel_grid_downsample(self, points,colors, voxel_size = 0.02):
        # use indices of voxel to re_represent points
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        unique_voxels, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)
        # mean of points in voxel, found by inverse indices
        downsampled_points = np.array([points[inverse_indices == i].mean(axis=0) for i in range(len(unique_voxels))])
        downsampled_colors = np.array([colors[inverse_indices == i].mean(axis=0) for i in range(len(unique_voxels))], dtype=colors.dtype)
        return downsampled_points, downsampled_colors

    @time_it
    def voxel_grid_downsample_optimized(self, points, colors, voxel_size=0.02):
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        
        unique_voxels, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)
        
        sum_points = np.zeros((len(unique_voxels), points.shape[1]), dtype=np.float32)
        sum_colors = np.zeros((len(unique_voxels), colors.shape[1]), dtype=np.float32)
        voxel_counts = np.zeros(len(unique_voxels), dtype=np.int32)
        
        np.add.at(sum_points, inverse_indices, points)
        np.add.at(sum_colors, inverse_indices, colors)
        np.add.at(voxel_counts, inverse_indices, 1)
        
        downsampled_points = sum_points / voxel_counts[:, None]
        downsampled_colors = sum_colors / voxel_counts[:, None]
        
        return downsampled_points, downsampled_colors

    def publish_transform(self, child_frame_id, pose, father_frame_id = 'camera_depth_optical_frame'):
            transform = TransformStamped()

            transform.header.stamp = self.get_clock().now().to_msg()

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

@time_it
def create_pointcloud2(points, colors, frame_id= None, stamp = None):
    assert points.shape[0] == colors.shape[0], "Points and colors must have the same number of elements."

    num_points = points.shape[0]
    pointcloud_data = []

    for i in range(num_points):
        x, y, z = points[i]
        
        r, g, b = (colors[i] * 255).astype(np.uint8)  
        
        rgb = (r << 16) | (g << 8) | b  

        pointcloud_data.append(struct.pack('fffI', x, y, z, rgb))

    pointcloud_data = b''.join(pointcloud_data)

    # create pointcloud msg
    header = std_msgs.msg.Header()
    header.stamp = stamp  
    header.frame_id = frame_id
    pointcloud_msg = pc2.create_cloud_xyz32(header, points)
    pointcloud_msg.header = header
    pointcloud_msg.height = 1
    pointcloud_msg.width = num_points
    pointcloud_msg.fields = [
        pc2.PointField(name='x', offset=0, datatype=7, count=1),
        pc2.PointField(name='y', offset=4, datatype=7, count=1),
        pc2.PointField(name='z', offset=8, datatype=7, count=1),
        pc2.PointField(name='rgb', offset=12, datatype=7, count=1)
    ]
    pointcloud_msg.is_bigendian = False
    pointcloud_msg.point_step = 16  
    pointcloud_msg.row_step = pointcloud_msg.point_step * num_points
    pointcloud_msg.data = pointcloud_data
    pointcloud_msg.is_dense = True

    expected_data_size = pointcloud_msg.width * pointcloud_msg.height * pointcloud_msg.point_step
    actual_data_size = len(pointcloud_msg.data)
    assert expected_data_size == actual_data_size, f"Data size mismatch: expected {expected_data_size}, got {actual_data_size}"

    return pointcloud_msg

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[3] = cy * cp * cr + sy * sp * sr
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr

    return q

def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
