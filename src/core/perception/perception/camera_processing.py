#!/usr/bin/env python

import math
from math import atan2, sqrt, pi
import time
import numpy as np

import rclpy
import rclpy.duration
from rclpy.node import Node

from collections import Counter

import rclpy.time
from sensor_msgs.msg import PointCloud2
import std_msgs
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import Pose, do_transform_pose
import tf2_geometry_msgs

from .RANSAC import RANSACPlaneDetector as RSPD
from .DBSCAN import better_DBSCAN
from geometry_msgs.msg import PoseStamped, Point,PointStamped
from .quaternion_from_euler import quaternion_from_euler
from .RANSAClinedetector import RANSACLineDetector

import ctypes
import struct
from core_interfaces.srv import PointcloudDetect
from core_interfaces.msg import PointcloudDetectedObj
from core_interfaces.srv import LidarlikeFromCamera
from core_interfaces.srv import CameraDetect

time_it_enable = False

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

        self.detection_max_distance = 1.0
        self.pointcloud =None

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a transform listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the publisher
        self.pointcloud_pub1 = self.create_publisher(
            PointCloud2, '/camera/camera/depth/color/obj_pointcloud', 10)
        self.pointcloud_pub2 = self.create_publisher(
            PointCloud2, '/camera/camera/depth/color/lidarlike_pointcloud', 10)
        
        self.srv = self.create_service(PointcloudDetect, 'pointcloud_detect', self.pointcloud_detect_callback)

        self.srv = self.create_service(LidarlikeFromCamera, "lidarlike_from_camera", self.pointcloud_to_lidarlike_callback)

        self.srv = self.create_service(CameraDetect, "camera_detect", self.camera_detect_callback)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)
        
        self.create_subscription(
            PointCloud2, '/lidar_pointcloud', self.lidar_callback, 10)

    def lidar_callback(self, msg: PointCloud2):
        self.lidar_pointcloud = msg

    def cloud_callback(self, msg: PointCloud2):
        self.pointcloud = msg

    def pointcloud_detect_callback(self, request, response):
        """
        Takes point cloud readings to detect objects.
        """
        def list_to_response(objects, boxes):
            response = PointcloudDetect.Response()
            for posestamped in objects:
                assert isinstance(posestamped,PoseStamped), "pose in obj is not a posestamped"
                detectedobj=PointcloudDetectedObj()
                detectedobj.category="item"
                detectedobj.pose=posestamped
                response.objects.append(detectedobj)
            for posestamped in boxes:
                assert isinstance(posestamped,PoseStamped), "pose in box is not a posestamped"
                detectedbox=PointcloudDetectedObj()
                detectedbox.category="box"
                detectedbox.pose=posestamped
                response.objects.append(detectedbox)
            # print(response)
            return response

        assert self.pointcloud!=None,"pointcloud detect error, no pointcloud msg is published, plz run 'camera_on'"
        assert isinstance(request.target_frame,str),"request type error, target frame is not a string"

        self.time_request=self.get_clock().now()
        self.stamp = self.time_request.to_msg()

        self.objects = []
        self.boxes = [] 

        # here start the show-----------------------------------------------------------------

        # faster pc to numpy
        points, colors = self.pointcloud_to_points_and_colors_optimized(self.pointcloud)

       # faster downsample
        points, colors = self.voxel_grid_downsample_optimized(points,colors, 0.01)

        #filter points for ground
        points, colors = self.filter_points(points, colors, min_height = 0.08, max_height = -100)

        #create pc for publishing 
        pointcloud2 = create_pointcloud2(points, colors, "camera_depth_optical_frame", self.stamp)
        self.pointcloud_pub1.publish(pointcloud2)

        cluster = self.cluster_points(points, colors)
        self.deal_with_clustered_points(cluster)
        #now we have a list of objs and boxes

        #transform to the frame in request
        transformed_objects = self.transform_list(self.objects,request.target_frame)
        transformed_boxes = self.transform_list(self.boxes,request.target_frame)

        response = list_to_response(transformed_objects,transformed_boxes)

        return response

    def pointcloud_to_lidarlike_callback(self, request, response):
        def transform_points_to_frame(points, transform, stamp):
            """
            transform a group of points together
            """
            transformed_points = []
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
            return transformed_points

        assert self.pointcloud!=None,"pointcloud detect error, no pointcloud msg is published, plz run 'camera_on'"
        assert isinstance(request.height,float),"request type error, height is not a float32"

        # self.time_request=self.get_clock().now()
        # self.stamp = self.time_request.to_msg()
        self.stamp = self.pointcloud.header.stamp
        _time = rclpy.time.Time.from_msg(self.pointcloud.header.stamp)

        # faster pc to numpy
        points, colors = self.pointcloud_to_points_and_colors_optimized(self.pointcloud)

       # faster downsample
        points, colors = self.voxel_grid_downsample_optimized(points,colors, 0.01)

        #filter points for ground
        points, colors = self.filter_points(points, colors, min_height = request.height + 0.02, max_height = request.height - 0.02)

        pointcloud2 = create_pointcloud2(points, colors, "camera_depth_optical_frame", self.stamp)
        self.pointcloud_pub.publish(pointcloud2)
        
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=request.target_frame,
            source_frame="camera_depth_optical_frame",
            time=_time,
        )
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)
        try:
            t = self.tf_buffer.lookup_transform(request.target_frame,  
                                                        "camera_depth_optical_frame",
                                                        _time)  
        except:
            self.get_logger().error(f"cannot transform to {request.target_frame} from 'lidar_link' with lookup time: {_time}")

        response = LidarlikeFromCamera.Response()
        response.points = transform_points_to_frame(points, t, self.stamp)
        
        
        return response

    def camera_detect_callback(self, request, response):
        def list_to_response(objects, boxes):
            def array_to_ros2_points(array):
                points_list = []
                for row in array:
                    point = Point()
                    point.x = row[0]
                    point.y = row[1]
                    point.z = row[2]
                    points_list.append(point)
                return points_list
            response = CameraDetect.Response()
            for row in objects:
                posestamped = row['ps']
                points = row['points']
                colors = row['colors']
                assert isinstance(posestamped,PoseStamped), "pose in obj is not a posestamped"
                detectedobj=PointcloudDetectedObj()
                detectedobj.category="item"
                detectedobj.pose=posestamped
                detectedobj.points = array_to_ros2_points(points)
                detectedobj.colors = array_to_ros2_points(colors)
                response.objects.append(detectedobj)
            for row in boxes:
                posestamped = row['ps']
                points = row['points']
                colors = row['colors']
                assert isinstance(posestamped,PoseStamped), "pose in box is not a posestamped"
                detectedbox=PointcloudDetectedObj()
                detectedbox.category="box"
                detectedbox.pose=posestamped
                detectedbox.points = array_to_ros2_points(points)
                detectedbox.colors = array_to_ros2_points(colors)
                response.objects.append(detectedbox)
            # print(response)
            return response
        
        def transform_points_to_frame(points, transform, stamp):
            """
            transform a group of points together
            """
            transformed_points = []
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
            return transformed_points

        start_time = time.time() 

        assert self.pointcloud!=None,"pointcloud detect error, no pointcloud msg is published, plz run 'camera_on'"
        self.get_logger().info("camera detecting ------------")

        self.stamp = self.pointcloud.header.stamp
        _time = rclpy.time.Time.from_msg(self.pointcloud.header.stamp)

        self.objects = []
        self.boxes = [] 

        # faster to numpy
        points, colors = self.pointcloud_to_points_and_colors_optimized(self.pointcloud)

        print("to numpy time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

       # faster downsample
        points, colors = self.voxel_grid_downsample_optimized(points,colors, 0.01)

        print("ds time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        points_upground, colors_upground = self.filter_points(points, colors, min_height = 0.08, max_height = 0.04)
        cluster = self.cluster_points(points_upground, colors_upground)
        self.deal_with_clustered_objs(cluster)

        pointcloud2obj = create_pointcloud2(points_upground, colors_upground, "camera_depth_optical_frame", self.stamp)
        self.pointcloud_pub1.publish(pointcloud2obj)

        print("g1 time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        points_likelidar, colors_likelidar = self.filter_points(points, colors, min_height = request.height + 0.015, max_height = request.height - 0.015)
        cluster = self.cluster_points(points_likelidar, colors_likelidar)
        self.deal_with_clustered_box(cluster)

        pointcloud2box = create_pointcloud2(points_likelidar, colors_likelidar, "camera_depth_optical_frame", self.stamp)
        self.pointcloud_pub2.publish(pointcloud2box)

        print("g2 time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        transformed_objects = self.transform_list(self.objects,_time,request.target_frame)
        transformed_boxes = self.transform_list(self.boxes,_time,request.target_frame)
        response = list_to_response(transformed_objects,transformed_boxes)
        
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=request.target_frame,
            source_frame="camera_depth_optical_frame",
            time=_time,
        )
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)
        try:
            t = self.tf_buffer.lookup_transform(request.target_frame,  
                                                        "camera_depth_optical_frame",
                                                        _time)  
        except:
            self.get_logger().error(f"cannot transform to {request.target_frame} from 'lidar_link' with lookup time: {_time}")

        response.points = transform_points_to_frame(points_likelidar, t, self.stamp)


        print("ending time consumption:")
        print(time.time() - start_time)  
        start_time = time.time() 

        return response

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
    def filter_points(self, points, colors, min_height=0.08, max_height=0.06):
        # print(f"Original points shape: {points.shape}")
        # print(f"Original colors shape: {colors.shape}")

        mask_min = points[:, 1] <= min_height
        filtered_points = points[mask_min]
        filtered_colors = colors[mask_min]

        # print(f"Filtered points shape after min_height filter: {filtered_points.shape}")
        # print(f"Filtered colors shape after min_height filter: {filtered_colors.shape}")

        mask_max = filtered_points[:, 1] >= max_height
        filtered_points = filtered_points[mask_max]
        filtered_colors = filtered_colors[mask_max]

        # print(f"Filtered points shape after max_height filter: {filtered_points.shape}")
        # print(f"Filtered colors shape after max_height filter: {filtered_colors.shape}")

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

    #abandoned
    def deal_with_clustered_points(self, cluster):
        def calc_plane_center(center1,normal1,center2,normal2):
            # we just take 0 as x and 2(z) as y and the real y is fixed
            A1, B1 = normal1[2], -normal1[0]
            C1 = A1*center1[0] + B1*center1[2]
            A2, B2 = normal2[2], -normal2[0]
            C2 = A2*center2[0] + B2*center2[2]

            A = np.array([[A1, B1], [A2, B2]]) 
            B = np.array([C1, C2])  

            intersection = np.linalg.solve(A, B)

            pose = PoseStamped()
            pose.header.stamp = self.stamp
            pose.header.frame_id = "camera_depth_optical_frame"
            pose.pose.position.x = intersection[0]
            pose.pose.position.y = center1[1]
            pose.pose.position.z = intersection[1]

            return pose
        def plane_to_pose(plane_params):
            center = plane_params[:3] 
            normal = plane_params[3:]  

            center[1] = 0.04

            if normal[2] > 0:  
                normal = -normal  

            roll = 0.0
            pitch = -atan2(normal[2], normal[0]) 
            yaw = 0.0  
            
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            pose = PoseStamped()
            pose.header.stamp = self.stamp
            pose.header.frame_id = "camera_depth_optical_frame"
            pose.pose.position.x = center[0]
            pose.pose.position.y = center[1]
            pose.pose.position.z = center[2]
            
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            
            return pose, center, normal
                
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
                        pose, center, normal = plane_to_pose(plane)
                        center_list.append(center)
                        normal_list.append(normal)
                        # self.publish_transform(f"box_face_{i}", pose)
                    pose = calc_plane_center(center_list[0],normal_list[0],center_list[1],normal_list[1])
                    # self.publish_transform(f"box_center",pose)
                    self.boxes.append(pose)
                else:
                    for i, plane in enumerate(planes):
                        pose, center, normal = plane_to_pose(plane)
                        # self.publish_transform(f"box_{i}",pose)
                        self.boxes.append(pose)                   

            else:                
                pose = PoseStamped()
                pose.header.stamp = self.stamp
                pose.header.frame_id = "camera_depth_optical_frame"
                pose.pose.position.x = avg_position[0]
                pose.pose.position.y = avg_position[1]
                pose.pose.position.z = avg_position[2]
                # self.publish_transform(f"obj_{cluster_id}", pose)
                self.objects.append(pose)

    @time_it
    def deal_with_clustered_objs(self, cluster):
        if not cluster:
            self.get_logger().info("No clusters found. Returning without processing.")
            return
        
        for cluster_id, cluster_data in cluster.items():
            points = cluster_data['points']
            colors = cluster_data['colors']

            avg_position = np.mean(points, axis=0)
            avg_color = np.mean(colors, axis=0)
          
            if len(points) > 90:
                # self.get_logger().info(f"obj clustering function skip one detected box, point_num: {len(points)}")
                pass
            elif len(points) > 6:
                pose = PoseStamped()
                pose.header.stamp = self.stamp
                pose.header.frame_id = "camera_depth_optical_frame"
                pose.pose.position.x = avg_position[0]
                pose.pose.position.y = avg_position[1]
                pose.pose.position.z = avg_position[2]
                self.get_logger().info(f"find obj_{cluster_id}")

                # self.publish_transform(f"obj_{cluster_id}", pose)
                # self.objects.append(pose)
                self.objects.append({'ps': pose, 'points': points, 'colors': colors })

            else:
                self.get_logger().info("too little points clustered, ignore once")

    @time_it
    def deal_with_clustered_box(self, cluster):
        if not cluster:
            self.get_logger().info("No clusters found. Returning without processing.")
            return
        
        for cluster_id, cluster_data in cluster.items():
            points = cluster_data['points']
            colors = cluster_data['colors']
            points2d = points[:, [0, 2]]
          
            if len(points2d) > 30:
                line_detector = RANSACLineDetector(ransac_n=2, max_dst=0.01, max_trials=200, stop_inliers_ratio=0.9)
                line_set, line_inliers_set, remains = line_detector.ransac_line_detection(points2d)
                
                # line with more inliers
                max_inliers = -1
                best_line = None
                best_inliers = None
                
                for index, line in enumerate(line_set):
                    inliers = line_inliers_set[index]
                    if len(inliers) > max_inliers:
                        max_inliers = len(inliers)
                        best_line = line
                        best_inliers = inliers # find line with most inliers
                
            
                p1, p2 = best_line  # line parameters are two points (p1, p2)
                # direction vector
                direction_vector = p2 - p1
                direction_vector_norm = np.linalg.norm(direction_vector)
                direction_vector_normalized = direction_vector / direction_vector_norm
                if direction_vector_normalized[1] < 0:
                    direction_vector_normalized = -direction_vector_normalized
                
                # prependicular vector
                perpendicular_vector = np.array([-direction_vector_normalized[1], direction_vector_normalized[0]])
                if perpendicular_vector[1] < 0:
                    perpendicular_vector = -perpendicular_vector

                # center point 
                center_point = np.mean(best_inliers, axis=0)

                # find edge points
                projections = np.dot(best_inliers - p1, direction_vector_normalized)
                min_projection_index = np.argmin(projections)
                max_projection_index = np.argmax(projections)
                min_point = best_inliers[min_projection_index]
                max_point = best_inliers[max_projection_index]

                # length
                line_length = np.linalg.norm(max_point - min_point)

                # 打印相关信息
                # self.get_logger().info(f"Cluster {cluster_id}")
                # self.get_logger().info(f"  Direction vector: {direction_vector}")
                # self.get_logger().info(f"  Perpendicular vector: {perpendicular_vector}")
                # self.get_logger().info(f"  Line center: {center_point}")
                # self.get_logger().info(f"Line length: {line_length}")

                if 0.1 <= line_length <= 0.3:
                    if 0.1 <= line_length <= 0.2: # box line 16
                        box_center = center_point + 0.12 * perpendicular_vector  
                        box_orientation = perpendicular_vector  
                        self.get_logger().info(f"Box center: {box_center}")
                        self.get_logger().info(f"Box orientation: {box_orientation}")
                    
                    elif 0.2 <= line_length <= 0.3: # box line 24
                        box_center = center_point + 0.08 * perpendicular_vector  
                        box_orientation = direction_vector_normalized 
                        self.get_logger().info(f"Box center: {box_center}")
                        self.get_logger().info(f"Box orientation: {box_orientation}")
                    
                    roll = 0.0
                    pitch = -atan2(box_orientation[1], box_orientation[0]) 
                    if pitch > pi/2:
                        pitch -= pi
                    elif pitch < -pi/2:
                        pitch += pi
                    yaw = 0.0  
                    quaternion = quaternion_from_euler(roll, pitch, yaw)

                    pose = PoseStamped()
                    pose.header.stamp = self.stamp
                    pose.header.frame_id = "camera_depth_optical_frame"
                    pose.pose.position.x = box_center[0]
                    pose.pose.position.y = 0.05
                    pose.pose.position.z = box_center[1]
                    
                    pose.pose.orientation.x = quaternion[0]
                    pose.pose.orientation.y = quaternion[1]
                    pose.pose.orientation.z = quaternion[2]
                    pose.pose.orientation.w = quaternion[3]

                    self.get_logger().info("find one box")
                    # self.publish_transform("box", pose)
                    # self.boxes.append(pose)
                    self.boxes.append({'ps': pose, 'points': points, 'colors': colors})

                else:
                    self.get_logger().warning(f"Error: Detected line length {line_length} does not match expected box dimensions (10-30).")
                    return  
                    
            else:
                # self.get_logger().info(f"box clustering function skip detected obj, point_num: {len(points)}")
                pass

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

    def transform_list(self, list: list[PoseStamped], transform_time, to_frame, from_frame="camera_depth_optical_frame"):
        new_list=[]
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame,
            source_frame=from_frame,
            time=transform_time,
        )
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)
        try:
            transform = self.tf_buffer.lookup_transform(to_frame,  
                                                        from_frame,
                                                        transform_time)  
        except:
            self.get_logger().error(f"cannot transform to {to_frame} from {from_frame} with lookup time: {transform_time}")
        for row in list:
            posestamped = row['ps']
            points = row['points']
            colors = row['colors']
            transformed_pose=tf2_geometry_msgs.do_transform_pose(posestamped.pose, transform)
            transformed_posestamped = PoseStamped()
            transformed_posestamped.header.stamp = posestamped.header.stamp 
            transformed_posestamped.header.frame_id = transform.header.frame_id 
            transformed_posestamped.pose = transformed_pose 

            tpoints, tcolors = self.transform_points_and_colors(points, colors, transform)
            # pointcloud = create_pointcloud2(tpoints, tcolors, "camera_depth_optical_frame", self.stamp)
            new_list.append({'ps':transformed_posestamped, 'points': tpoints, 'colors': tcolors})
        return new_list
    
    def transform_points_and_colors(self, points, colors, transform):
        transformed_points = []

        for i in range(len(points)):
            x, y, z = points[i]

            point_pose = tf2_geometry_msgs.Pose()
            point_pose.position.x = x
            point_pose.position.y = y
            point_pose.position.z = z

            td_pointpose = tf2_geometry_msgs.do_transform_pose(point_pose, transform)

            transformed_points.append([td_pointpose.position.x, td_pointpose.position.y, td_pointpose.position.z])

        return transformed_points, colors

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
    pointcloud_msg =pc2.create_cloud_xyz32(header, points) 
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
