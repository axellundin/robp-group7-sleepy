#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sklearn.cluster import DBSCAN
from collections import Counter

from sensor_msgs.msg import PointCloud2
import std_msgs
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import Pose, do_transform_pose
from .ransac import RANSACPlaneDetector as rspd

import ctypes
import struct

from core_interfaces.msg import DetectedMsg

class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        '''
        self.objects = []
        self.boxes = [] 

        msg = DetectedMsg() 
        msg.objects = self.objects
        msg.boxes = self.boxes
        self.detected_objects_publisher = self.create_publisher(
            DetectedMsg,
            'detected',
            10
        '''
        self.detection_max_distance = 1.5

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

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        # Convert ROS -> NumPy
        points, colors = self.pointcloud_to_points_and_colors(msg)

        points, colors = self.voxel_grid_downsample(points,colors, 0.01)

        points, colors = self.filter_points(points, colors, 0.5)

        pointcloud2 = create_pointcloud2(points, colors, "camera_link", msg.header.stamp)
        self.pointcloud_pub.publish(pointcloud2)

        #do things here 

        # rspd.ransac_plane_detection(1,1,1)


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

    def filter_points(self, points, colors, min_height = 0.2):
        avg_x = np.mean(points[:, 0]) 
        avg_y = np.mean(points[:, 1]) 
        avg_z = np.mean(points[:, 2])  
        print(f"Average X: {avg_x}, Average Y: {avg_y}, Average Z: {avg_z}")

        filtered_points = points[points[:, 2] >= min_height]
        filtered_colors = colors[points[:, 2] >= min_height]
        return filtered_points, filtered_colors
        
    def cluster_points(self, points, colors, eps=1, min_samples=0.2):
        """
        use skleaarn.cluster dbscan 
        if a point have lots of points around it, it is considered as part of the point class
        """
        db = DBSCAN(eps=eps, min_samples=min_samples)  
        labels = db.fit_predict(points)
        label_counts = Counter(labels)

        for label, count in label_counts.items():
            if label == -1:
                print(f"Noise (label {label}) has {count} points.")
            else:
                print(f"Cluster {label} has {count} points.")

    def voxel_grid_downsample(self, points,colors, voxel_size):
        # use indices of voxel to re_represent points
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        unique_voxels, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)
        # mean of points in voxel, found by inverse indices
        downsampled_points = np.array([points[inverse_indices == i].mean(axis=0) for i in range(len(unique_voxels))])
        downsampled_colors = np.array([colors[inverse_indices == i].mean(axis=0) for i in range(len(unique_voxels))], dtype=colors.dtype)
        return downsampled_points, downsampled_colors

def create_pointcloud2(points, colors, frame_id= None, stamp = None):
    assert points.shape[0] == colors.shape[0], "Points and colors must have the same number of elements."

    num_points = points.shape[0]
    pointcloud_data = []

    for i in range(num_points):
        x, y, z = points[i]
        
        r, g, b = (colors[i] * 255).astype(np.uint8)  # 假设 colors 是 [0, 1] 范围的浮点数
        
        rgb = (r << 16) | (g << 8) | b  # RGB 转换为单个整数

        pointcloud_data.append(struct.pack('fffI', x, y, z, rgb))

    # 将所有点的数据合并为一个字节数组
    pointcloud_data = b''.join(pointcloud_data)

    # create pointcloud msg
    header = std_msgs.msg.Header()
    header.stamp = stamp  # 当前时间戳
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














'''
import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        # Convert ROS -> NumPy
        print("inne")

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255
#rod over gron + blo
#gron over rod + 48, blo over rod + 40

        distances = np.linalg.norm(points, axis=1)

        red = '\033[91m'
        green = '\033[92m'
        reset = '\033[0m'
        new_data = []
        for i in range(msg.width):
            keep_point = False
            if distances[i] < 0.9:
                if colors[i,0] > colors[i,1]+colors[i,2]:
                    keep_point = True
                    self.get_logger().info(f"{red}Detected red Sphere!{reset}")

                elif colors[i,1]>colors[i,0]+0.188 and colors[i,2]>colors[i,0]+0.157:
                    keep_point = True
                    self.get_logger().info(f"{green}Detected green Cube!{reset}")
            if keep_point == True:
                new_data.append(msg.data[i*msg.point_step:i*msg.point_step+msg.point_step])
            

        new_message = PointCloud2()
        new_message.fields = msg.fields
        new_message.header = msg.header
        new_message.height = 1
        new_message.is_bigendian = msg.is_bigendian
        new_message.point_step = msg.point_step
        new_message.is_dense = msg.is_dense
        new_message.width = len(new_data)
        new_message.row_step = new_message.point_step*new_message.width 
        new_message.data = b''.join(new_data)


        self._pub.publish(new_message)



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


the structure for /camera/camera/depth/color/points
header:
  stamp:
    sec: 1738691132
    nanosec: 359928955
  frame_id: camera_depth_optical_frame
height: 1
width: 103074
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
- name: rgb
  offset: 16
  datatype: 7
  count: 1
is_bigendian: false
point_step: 20
row_step: 6144000
data:
- 163
- 50
- 10
- 192
- 164
- '...'
is_dense: true




        red_ball_success, red_ball_center = self.detect_red_ball(colors, points)
        if red_ball_success:
            print("Detected red ball")
            self.send_detection_to_tf(msg.header.frame_id, red_ball_center, msg.header.stamp, "red_ball")

        green_cube_success, green_cube_center = self.detect_green_cube(colors, points)
        if green_cube_success:
            print("Detected green cube")
            self.send_detection_to_tf(msg.header.frame_id, green_cube_center, msg.header.stamp, "green_cube")

    def detect_red_ball(self, colors, points):
        discretization_length = 0.07
        neighborhood_distance = 0.1
        count_threshold = 10
        conditions = [1, -1, -1]
        color_value_thresholds = [0.6, 0.4, 0.4]
        return self.detect_obj(colors, points, discretization_length, neighborhood_distance, count_threshold, conditions, color_value_thresholds)
    
    def detect_green_cube(self, colors, points):
        discretization_length = 0.07
        neighborhood_distance = 0.1
        count_threshold = 10
        conditions = [-1, 1, 1]
        color_value_thresholds = [0.3, 0.5, 0.4]
        return self.detect_obj(colors, points, discretization_length, neighborhood_distance, count_threshold, conditions, color_value_thresholds)
    
    def detect_obj(self, colors, points, discretization_length, neighborhood_distance, count_threshold, conditions, color_value_thresholds):
        green_idx = self.filter_based_on_color(colors, conditions, color_value_thresholds)
        if len(green_idx) == 0:
            return False, None
        filtered_pc = points[green_idx]
        bounds = self.find_bounds_of_filtered_points(filtered_pc)
        success, max_center = self.count_points_into_histogram(filtered_pc, bounds, discretization_len=discretization_length, count_threshold=count_threshold)
        if not success: 
            return False, None
        fom = self.compute_first_order_moment(filtered_pc, max_center, neighborhood_distance=neighborhood_distance)
        print(f"{fom=}")

        return True, fom
    
    def send_detection_to_tf(self, camera_frame, position_in_camera_frame, timestamp, frame_name):

        frame_to='map'
        frame_from=camera_frame
        object_frame = f'objects/detected_{frame_name}'
        pose = Pose()
        pose.position.x = position_in_camera_frame[0]
        pose.position.y = position_in_camera_frame[1]
        pose.position.z = position_in_camera_frame[2]
        # Wait for the transform asynchronously
        _time = rclpy.time.Time().from_msg(timestamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=frame_to,
            source_frame=frame_from,
            time=_time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try: 
            T = self.tf_buffer.lookup_transform(
                frame_to, 
                frame_from, 
                time=_time, 
                # timeout=rclpy.time.Duration(seconds=5)
            )

            pose_in_map = do_transform_pose(pose, T)

            new_T = TransformStamped()
            new_T.child_frame_id = object_frame
            new_T.header.frame_id = frame_to 
            new_T.header.stamp = timestamp
            new_T.transform.translation.x = pose_in_map.position.x
            new_T.transform.translation.y = pose_in_map.position.y
            new_T.transform.translation.z = pose_in_map.position.z

            new_T.transform.rotation.x = pose_in_map.orientation.x
            new_T.transform.rotation.y = pose_in_map.orientation.y
            new_T.transform.rotation.z = pose_in_map.orientation.z
            new_T.transform.rotation.w = pose_in_map.orientation.w
            self.tf_broadcaster.sendTransform(new_T)
        except Exception as e: 
            print(str(e))

    def filter_based_on_color(self, colors, condition_directions, color_value_thresholds):
        filtered_idx = np.where(((colors[:, 0] - color_value_thresholds[0]) * condition_directions[0] > 0) 
                           & ((colors[:, 1] - color_value_thresholds[1]) * condition_directions[1] > 0)  
                           & ((colors[:, 2] - color_value_thresholds[2]) * condition_directions[2] > 0) )[0]
        return filtered_idx

    def find_bounds_of_filtered_points(self, filtered_pc):
        lower_bounds = np.min(filtered_pc, axis=0)
        upper_bounds = np.max(filtered_pc, axis=0)
        return np.vstack((lower_bounds, upper_bounds))

    def count_points_into_histogram(self, filtered_pc, bounds, discretization_len=0.1, count_threshold=5):
        # Compute number of bins: 
        n_bins = np.ceil((bounds[1,:] - bounds[0,:]) / discretization_len)
        n_bins = n_bins.astype(int)
        if np.min(n_bins) == 0:
            return False, None
        # Create histogram 
        hist = np.zeros(n_bins)
        # Count points into the bins         
        max_count = 0
        max_bin = None
        for point in filtered_pc:
            bin = np.floor((point - bounds[0,:]) / discretization_len)
            if np.min(n_bins - bin) == 0:
                print("Bad indicies. Skipping this point.")
                continue
            bin = tuple(bin.astype(int))
            hist[bin] += 1
            if hist[bin] > max_count:
                max_count = hist[bin]
                max_bin = bin 

        # Nothing found if max count is below threshold
        if max_count < count_threshold:
            return False, None 

        # Compute center of the bin: 
        center = bounds[0,:] + np.array(max_bin) * discretization_len + np.ones(3) * discretization_len / 2
        
        return True, center
        
    def compute_first_order_moment(self, filtered_pc, max_center, neighborhood_distance=0.3):
        # Filter points based on distance to center
        neighbor_points = filtered_pc[np.sqrt(np.sum((filtered_pc-max_center)**2, axis=1)) < neighborhood_distance]
        return np.mean(neighbor_points, axis=0)

    def _recursive_3d_search(self, idx_list, points, lim_min, lim_max, min_size=0.1):
        
        octant_count = np.zeros((2, 2, 2))
        lim_mid = lim_min + (lim_max - lim_min) // 2 
        new_idx_lst = []
        for idx in idx_list:
            p = points[idx,:]
            # Check if within bounds 
            if np.all(p - lim_min < 0) or np.all(p - lim_max > 0):
                continue

            # Find octant that it belongs to 
            octant = np.greater(p, lim_mid).astype(int)
            octant_count[octant[0], octant[1], octant[2]] += 1
            new_idx_lst.append(idx)

        new_idx_lst = np.array(new_idx_lst) 
        print(f"{octant_count=}")
        max_idx = np.where(octant_count == np.max(octant_count))
        max_idx = np.array([max_idx[0][0], max_idx[1][0], max_idx[2][0]])
        print(f"{max_idx=}")
        # Compute new limits 
        new_xlim_min = lim_min + max_idx * (lim_max - lim_min) // 2  
        new_xlim_max = lim_max - (np.ones(3) - max_idx) * (lim_max - lim_min) // 2  
        
        if np.max(new_xlim_max - new_xlim_min) < min_size:
            return len(new_idx_lst) > 5, points[new_idx_lst]

        return self._recursive_3d_search(new_idx_lst, points, lim_min, lim_max, min_size=min_size)
"""

'''