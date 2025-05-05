# import sys
# print("Running with Python:", sys.executable)
# import numpy as np
# import open3d as o3d
# import rclpy
# from rclpy.node import Node
# import tf2_ros
# from geometry_msgs.msg import TransformStamped
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import LaserScan
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from scipy.spatial import cKDTree
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Header
# import sensor_msgs_py.point_cloud2 as pc2
# from sensor_msgs.msg import PointField
# from std_msgs.msg import Float32
# from visualization_msgs.msg import Marker


# # Convert LiDAR scan to Open3D point cloud
# def convert_scan_to_open3d(scan_msg):
#     """Converts a ROS2 LaserScan message into an Open3D point cloud."""
#     # Get all valid range values and corresponding angles
#     ranges = np.array(scan_msg.ranges)
#     angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

#     valid = np.isfinite(ranges)
#     ranges = ranges[valid]
#     angles = angles[valid]

#     # Convert to Cartesian coordinates
#     x = ranges * np.cos(angles)
#     y = ranges * np.sin(angles)
#     points = np.column_stack((x, y))

#     # --- Region filter parameters ---
#     a = 0.085
#     b = 0.12
#     r = 0.39
#     q = 0.31

#     # Apply region mask
#     x_scaled = (points[:, 0] + a) / r
#     y_scaled = (points[:, 1] + b) / q
#     mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) >= 1
#     filtered_points = points[mask]

#     # KDTree-based neighbor filtering
#     radius = 0.01  # 1 cm neighborhood radius
#     kdtree = cKDTree(filtered_points)
#     to_keep = np.ones(filtered_points.shape[0], dtype=bool)

#     for i, p in enumerate(filtered_points):
#         if not to_keep[i]:
#             continue  # Already marked for removal
#         idxs = kdtree.query_ball_point(p, r=radius)
#         idxs.remove(i)  # Don't remove itself
#         to_keep[idxs] = False  # Remove all close neighbors

#     final_points = filtered_points#[to_keep]
#     pointcloud_xyz = np.column_stack((final_points, np.zeros(len(final_points))))
#     # Convert to Open3D point cloud
#     pc = o3d.geometry.PointCloud()
#     pc.points = o3d.utility.Vector3dVector(pointcloud_xyz)
#     return pc

# def open3d_to_pointcloud2(o3d_pc, frame_id="map"):
#     """Convert Open3D point cloud to ROS2 PointCloud2 message."""
#     points = np.asarray(o3d_pc.points)

#     header = Header()
#     header.stamp = rclpy.time.Time().to_msg()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#     ]

#     pc2_msg = pc2.create_cloud(header, fields, points)
#     return pc2_msg

# def open3d_to_colored_pointcloud2(o3d_pc, rgb_color=(255, 255, 255), frame_id="map"):
#     """Convert Open3D point cloud to a colored PointCloud2 message."""
#     points = np.asarray(o3d_pc.points)
#     r, g, b = rgb_color

#     # Encode RGB as packed float
#     rgb_floats = np.array([
#         np.frombuffer(np.uint32((r << 16) | (g << 8) | b).tobytes(), dtype=np.float32)[0]
#         for _ in range(len(points))
#     ])

#     # Combine (x, y, z, rgb)
#     colored_points = np.hstack((points, rgb_floats.reshape(-1, 1)))

#     header = Header()
#     header.stamp = rclpy.time.Time().to_msg()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
#     ]

#     return pc2.create_cloud(header, fields, colored_points)



# class IcpMappingNode(Node):
#     def __init__(self):
#         super().__init__('icp_mapping_node')
#         print("yassir mapping")
#         group = ReentrantCallbackGroup()
#         # Subscribe to the LiDAR scan topic
#         self.subscription = self.create_subscription(
#             LaserScan, '/scan', self.scan_callback, 1, callback_group=group)
        
#         # Subscribe to angular velocity
#         self.create_subscription(Float32, '/angular_velocity', self.omega_callback, 10, callback_group=group)


#         self.first_scan_pub = self.create_publisher(PointCloud2, '/first_scan', 10)
#         self.ref_scan_pub = self.create_publisher(PointCloud2, '/reference_scan', 10)
#         self.new_scan_pub = self.create_publisher(PointCloud2, '/new_scan', 10)
#         self.new_scan_icp_pub = self.create_publisher(PointCloud2, '/new_scan_icp', 10)

#         self.marker_pub = self.create_publisher(Marker, '/reference_zones', 10)

#         # **TF Broadcaster instead of topic publisher**
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # TF buffer and listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

#         # Store the latest correction transform (initialize as identity)
#         self.latest_correction = None

#         self.accumulated_map = None  # Store the global map in `map` frame

#         self.reference_scan = None
#         self.count = 0
#         self.icp_publish_counter = 0
#         self.scan_counter = 0
#         self.pc_counter = 0
#         self.pc_buffer_size = 10
#         self.merged_pc = None
#         self.bad_match_counter = 0
#         self.first_scan = None
#         self.omega = 0.0  # Latest angular velocity
#         self.omega_threshold = 0.2  # rad/s
#         self.reference_scans = []  # List of (pointcloud, (x, y)) s
#         self.reference_radius = 2.5  # meters
#         self.last_used_ref_index = -1


#     def scan_callback(self, msg):
#         """Transforms new scan to map frame, performs ICP, and updates transform."""
#         self.scan_counter += 1.0

#         if self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time(seconds=0)):
#             # self.get_logger().info("odom ready, icp intiated")

#         else:
#             # self.get_logger().warn("TF odom-base_link lookup failed for first scan; skipping scan, if you see this at the start=ok, otherwise problem")
#             return

#         if self.latest_correction is None:
#             self.latest_correction = np.eye(4)
#             self.publish_icp_transform(self.latest_correction, msg)
#             return
        
#         if self.first_scan is None:
#             # Transform the first scan to the map frame
#             if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
#                 tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
#                 T_map_lidar = self.transform_to_matrix(tf_map_lidar)
#                 first_scan = convert_scan_to_open3d(msg)
#                 first_scan.transform(T_map_lidar)
#                 self.first_scan = first_scan
#             else:
#                 # self.get_logger().warn("TF map-lidar lookup failed for first scan; skipping setting it")

        
#         if abs(self.omega) > self.omega_threshold:
#             # self.get_logger().info(f"Skipping ICP — omega too high: {self.omega:.2f} rad/s")
#             self.publish_icp_transform(self.latest_correction, msg)
#             return


#         new_scan = convert_scan_to_open3d(msg)

#         if self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time(seconds=0)):
#             transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(seconds=0))
#             current_x = transform.transform.translation.x
#             current_y = transform.transform.translation.y
#         else:
#             # self.get_logger().warn("TF odom-base_link Lookup failed: skipping update")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return

#         current_pos = (current_x, current_y)
#         valid_reference, ref_index = self.find_valid_reference(current_pos)
#         self.publish_reference_zones(ref_index)

#         if ref_index != -1:
#             self.last_used_ref_index = ref_index


#         # Compute distance from the active reference
#         if valid_reference is not None and ref_index != -1:
#             ref_pos = self.reference_scans[ref_index][1]
#             dx = current_pos[0] - ref_pos[0]
#             dy = current_pos[1] - ref_pos[1]
#             distance = (dx**2 + dy**2)**0.5

#             # Interpolate fitness threshold based on distance
#             if distance <= 1.0:
#                 threshold_fitness = 0.6 - 0.15 * (distance / 1.0)  # From 0.6 to 0.45
#             elif distance <= self.reference_radius:
#                 threshold_fitness = 0.45 - 0.15 * ((distance - 1.0) / (self.reference_radius - 1.0))  # From 0.45 to 0.3
#             else:
#                 threshold_fitness = 0.3  # Max looseness at edge or beyond
#         else:
#             threshold_fitness = 0.6  # Default for new reference    


#         # If no valid reference is available for the current position,
#         # then create a new reference scan.
#         if valid_reference is None:
#             # Transform new scan into odom frame
#             if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
#                 tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#                 T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)
#             else:
#                 # self.get_logger().warn("TF odom-lidar lookup failed for mini-ICP; skipping update")
#                 self.publish_icp_transform(self.latest_correction, msg)
#                 return

#             new_scan.transform(T_odom_lidar)

#             # Use last successfully used reference for mini-ICP refinement
#             closest_ref = None
#             if 0 <= self.last_used_ref_index < len(self.reference_scans):
#                 closest_ref = self.reference_scans[self.last_used_ref_index][0]

#             if closest_ref is not None:
#                 threshold = 0.12
#                 icp_mini_result = o3d.pipelines.registration.registration_icp(
#                     new_scan, closest_ref, threshold, self.latest_correction,
#                     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25000)
#                 )
#                 # self.get_logger().info(f"[MiniICP] Fitness: {icp_mini_result.fitness:.3f} | RMSE: {icp_mini_result.inlier_rmse:.3f}")
#                 if icp_mini_result.fitness > 0.6:
#                     # self.get_logger().info("Mini-ICP applied to refine new reference scan.")
#                     self.latest_correction = icp_mini_result.transformation
#                 else:
#                     # self.get_logger().warn("Mini-ICP rejected due to low fitness.")

#             # Transform refined scan into map frame
#             T_map_odom = self.latest_correction
#             new_scan.transform(T_map_odom)

#             # Store the new reference
#             self.reference_scans.append((new_scan, current_pos))
#             valid_reference = new_scan

#             # Publish for visualization
#             ref_msg = open3d_to_pointcloud2(valid_reference)
#             self.ref_scan_pub.publish(ref_msg)
#             return

        
#         if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
#             tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#             # print(tf_map_lidar)
#             ## self.get_logger().info(tf_odom_lidar)
#             T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)
#             ## self.get_logger().info(tf_odom_lidar)

#         else:
#             # self.get_logger().warn("TF odom-lidar Lookup failed: skipping update")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return
        
#         # Transform new scan to the map frame
#         new_scan.transform(T_odom_lidar)
#         # if self.count == 10:
#         new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(0, 255, 0))  # green
#         self.new_scan_pub.publish(new_msg)


#         # ICP Convergence threshold
#         threshold = 0.12
#         # Run ICP: Align `new_scan` (source) to `accumulated_map` (target)
#         icp_result = o3d.pipelines.registration.registration_icp(
#             new_scan, valid_reference, threshold, self.latest_correction,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=12000))
        

#         #consider using fitness to choose activ/new reference frame
#         # self.get_logger().info(f'inlier: {icp_result.inlier_rmse}, fitness: {icp_result.fitness}',)
#         if icp_result.inlier_rmse > 0.07 or icp_result.fitness < threshold_fitness:
#             # self.get_logger().warn("ICP registration did not match well, skipping update.")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return

#         # Update the correction transform
#         self.latest_correction = icp_result.transformation
       
        
#         self.icp_publish_counter += 1.0
#         if self.icp_publish_counter % 1.0 == 0.0:
#             # self.get_logger().info("uppdate")
#             self.publish_icp_transform(self.latest_correction, msg)

#             #consider replacing this with the transform from the icp directly after debugging
#             if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(seconds=0)):
#                 tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
#                 T_map_odom = self.transform_to_matrix(tf_map_odom)

#             else:
#                 # self.get_logger().warn("TF map-odom Lookup failed: skipping update")
#                 self.icp_publish_counter += 1.0
#                 if self.icp_publish_counter % 1.0 == 0.0:
#                     self.publish_icp_transform(self.latest_correction, msg)
#                 return
#             new_scan.transform(T_map_odom)

#             self.count = self.count + 1
#             new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(255, 0, 0))  #red
#             self.new_scan_icp_pub.publish(new_msg)
#             self.merged_pc = None
#             self.pc_counter = 0

#         # Always republish the first scan
#         if self.first_scan is not None:
#             first_msg = open3d_to_colored_pointcloud2(self.first_scan, rgb_color=(0, 0, 255))  # Blue
#             self.first_scan_pub.publish(first_msg)


#     def omega_callback(self, msg):
#         self.omega = msg.data

#     def find_valid_reference(self, current_pos):
#         for i, (ref_pc, ref_pos) in enumerate(self.reference_scans):
#             dx = current_pos[0] - ref_pos[0]
#             dy = current_pos[1] - ref_pos[1]
#             if (dx**2 + dy**2)**0.5 < self.reference_radius:
#                 return ref_pc, i  # Return the point cloud and index
#         return None, -1




#     def transform_to_matrix(self, tf_msg):
#         """Converts a ROS2 TransformStamped to a 4x4 transformation matrix."""
#         trans = tf_msg.transform.translation
#         rot = tf_msg.transform.rotation

#         # Convert quaternion to a 3x3 rotation matrix
#         r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

#         # Construct the 4x4 transformation matrix
#         matrix = np.eye(4)
#         matrix[:3, :3] = r
#         matrix[:3, 3] = [trans.x, trans.y, trans.z]

#         return matrix


#     def publish_reference_zones(self, active_index):
#         for i, (_, ref_pos) in enumerate(self.reference_scans):
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "reference_zones"
#             marker.id = i
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
#             marker.pose.position.x = ref_pos[0]
#             marker.pose.position.y = ref_pos[1]
#             marker.pose.position.z = 0.0  # 2D

#             marker.pose.orientation.w = 1.0  # No rotation

#             marker.scale.x = self.reference_radius * 2.0
#             marker.scale.y = self.reference_radius * 2.0
#             marker.scale.z = 0.01  # Thin cylinder

#             if i == active_index:
#                 # Active zone — green
#                 marker.color.r = 0.0
#                 marker.color.g = 1.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.5
#             else:
#                 # Inactive zone — red
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.2

#             marker.lifetime.sec = 0  # Keep until overwritten
#             self.marker_pub.publish(marker)


#     def publish_icp_transform(self, transform, msg):
#         """Publishes ICP correction directly to /tf."""
#         # get latest odom transform

#         odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0))
    
#         t = TransformStamped()
#         t.header.stamp = odom_transform.header.stamp
#         t.header.frame_id = 'map'
#         t.child_frame_id = 'odom'

#         t.transform.translation.x = transform[0, 3]
#         t.transform.translation.y = transform[1, 3]
#         t.transform.translation.z = 0.0  # 2D

#         r = R.from_matrix(transform[0:3, 0:3])
#         quat = r.as_quat()
#         t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat

#         # **Broadcast the transform to /tf**
#         self.tf_broadcaster.sendTransform(t)

#         ## self.get_logger().info("Published ICP correction to /tf.")

# def main():
#     rclpy.init()
#     node = IcpMappingNode()
#     ex = MultiThreadedExecutor()
#     ex.add_node(node)
#     ex.spin()
#     # rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
















# import sys
# print("Running with Python:", sys.executable)
# import numpy as np
# import open3d as o3d
# import rclpy
# from rclpy.node import Node
# import tf2_ros
# from geometry_msgs.msg import TransformStamped
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import LaserScan
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from scipy.spatial import cKDTree
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Header
# import sensor_msgs_py.point_cloud2 as pc2
# from sensor_msgs.msg import PointField
# from std_msgs.msg import Float32
# from visualization_msgs.msg import Marker


# # Convert LiDAR scan to Open3D point cloud
# def convert_scan_to_open3d(scan_msg):
#     """Converts a ROS2 LaserScan message into an Open3D point cloud."""
#     # Get all valid range values and corresponding angles
#     ranges = np.array(scan_msg.ranges)
#     angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

#     valid = np.isfinite(ranges)
#     ranges = ranges[valid]
#     angles = angles[valid]

#     # Convert to Cartesian coordinates
#     x = ranges * np.cos(angles)
#     y = ranges * np.sin(angles)
#     points = np.column_stack((x, y))

#     # --- Region filter parameters ---
#     a = 0.085
#     b = 0.12
#     r = 0.39
#     q = 0.31

#     # Apply region mask
#     x_scaled = (points[:, 0] + a) / r
#     y_scaled = (points[:, 1] + b) / q
#     mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) >= 1
#     filtered_points = points[mask]

#     # KDTree-based neighbor filtering
#     radius = 0.01  # 1 cm neighborhood radius
#     kdtree = cKDTree(filtered_points)
#     to_keep = np.ones(filtered_points.shape[0], dtype=bool)

#     for i, p in enumerate(filtered_points):
#         if not to_keep[i]:
#             continue  # Already marked for removal
#         idxs = kdtree.query_ball_point(p, r=radius)
#         idxs.remove(i)  # Don't remove itself
#         to_keep[idxs] = False  # Remove all close neighbors

#     final_points = filtered_points#[to_keep]
#     pointcloud_xyz = np.column_stack((final_points, np.zeros(len(final_points))))
#     # Convert to Open3D point cloud
#     pc = o3d.geometry.PointCloud()
#     pc.points = o3d.utility.Vector3dVector(pointcloud_xyz)
#     return pc

# def open3d_to_pointcloud2(o3d_pc, frame_id="map"):
#     """Convert Open3D point cloud to ROS2 PointCloud2 message."""
#     points = np.asarray(o3d_pc.points)

#     header = Header()
#     header.stamp = rclpy.time.Time().to_msg()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#     ]

#     pc2_msg = pc2.create_cloud(header, fields, points)
#     return pc2_msg

# def open3d_to_colored_pointcloud2(o3d_pc, rgb_color=(255, 255, 255), frame_id="map"):
#     """Convert Open3D point cloud to a colored PointCloud2 message."""
#     points = np.asarray(o3d_pc.points)
#     r, g, b = rgb_color

#     # Encode RGB as packed float
#     rgb_floats = np.array([
#         np.frombuffer(np.uint32((r << 16) | (g << 8) | b).tobytes(), dtype=np.float32)[0]
#         for _ in range(len(points))
#     ])

#     # Combine (x, y, z, rgb)
#     colored_points = np.hstack((points, rgb_floats.reshape(-1, 1)))

#     header = Header()
#     header.stamp = rclpy.time.Time().to_msg()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
#     ]

#     return pc2.create_cloud(header, fields, colored_points)



# class IcpMappingNode(Node):
#     def __init__(self):
#         super().__init__('icp_mapping_node')
#         print("yassir mapping")
#         group = ReentrantCallbackGroup()
#         # Subscribe to the LiDAR scan topic
#         self.subscription = self.create_subscription(
#             LaserScan, '/scan', self.scan_callback, 1, callback_group=group)
        
#         # Subscribe to angular velocity
#         self.create_subscription(Float32, '/angular_velocity', self.omega_callback, 10, callback_group=group)


#         self.first_scan_pub = self.create_publisher(PointCloud2, '/first_scan', 10)
#         self.ref_scan_pub = self.create_publisher(PointCloud2, '/reference_scan', 10)
#         self.new_scan_pub = self.create_publisher(PointCloud2, '/new_scan', 10)
#         self.new_scan_icp_pub = self.create_publisher(PointCloud2, '/new_scan_icp', 10)

#         self.marker_pub = self.create_publisher(Marker, '/reference_zones', 10)

#         # **TF Broadcaster instead of topic publisher**
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # TF buffer and listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

#         # Store the latest correction transform (initialize as identity)
#         self.latest_correction = None

#         self.accumulated_map = None  # Store the global map in `map` frame

#         self.reference_scan = None
#         self.count = 0
#         self.icp_publish_counter = 0
#         self.scan_counter = 0
#         self.pc_counter = 0
#         self.pc_buffer_size = 10
#         self.merged_pc = None
#         self.bad_match_counter = 0
#         self.first_scan = None
#         self.omega = 0.0  # Latest angular velocity
#         self.omega_threshold = 0.2  # rad/s
#         self.reference_scans = []  # List of (pointcloud, (x, y)) s
#         self.reference_radius = 2.5  # meters
#         self.last_used_ref_index = -1


#         # Special reference region config (x in meters)
#         self.special_ref_zone_xmin = 7.45
#         self.special_ref_zone_xmax = 8.25
#         self.special_ref_radius = 0.70 
#         self.special_references = []  # list of (pointcloud, (x, y))
#         self.use_special_refs = False




#     def scan_callback(self, msg):
#         """Transforms new scan to map frame, performs ICP, and updates transform."""
#         self.scan_counter += 1.0

#         if self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time(seconds=0)):
#             self.get_logger().info("odom ready, icp intiated")
#             pass

#         else:
#             self.get_logger().warn("TF odom-base_link lookup failed for first scan; skipping scan, if you see this at the start=ok, otherwise problem")
#             return

#         if self.latest_correction is None:
#             self.latest_correction = np.eye(4)
#             self.publish_icp_transform(self.latest_correction, msg)
#             return
        
#         if self.first_scan is None:
#             # Transform the first scan to the map frame
#             if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
#                 tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
#                 T_map_lidar = self.transform_to_matrix(tf_map_lidar)
#                 first_scan = convert_scan_to_open3d(msg)
#                 first_scan.transform(T_map_lidar)
#                 self.first_scan = first_scan
#             else:
#                 self.get_logger().warn("TF map-lidar lookup failed for first scan; skipping setting it")
#                 pass

        
#         if abs(self.omega) > self.omega_threshold:
#             self.get_logger().info(f"Skipping ICP — omega too high: {self.omega:.2f} rad/s")
#             self.publish_icp_transform(self.latest_correction, msg)
#             return


#         new_scan = convert_scan_to_open3d(msg)

#         if self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time(seconds=0)):
#             transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(seconds=0))
#             current_x = transform.transform.translation.x
#             current_y = transform.transform.translation.y
#         else:
#             self.get_logger().warn("TF odom-base_link Lookup failed: skipping update")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return

#         self.use_special_refs = self.special_ref_zone_xmin <= current_x <= self.special_ref_zone_xmax
#         current_pos = (current_x, current_y)

#         if self.use_special_refs:
#             # Try to find a valid special reference
#             valid_reference = None
#             ref_index = -1
#             for i, (ref_pc, ref_pos) in enumerate(self.special_references):
#                 dist = np.linalg.norm(np.array(current_pos) - np.array(ref_pos))
#                 if dist <= self.special_ref_radius:
#                     valid_reference = ref_pc
#                     ref_index = i  # optional: use for visualization
#                     break  # stop at the first valid one

#         else:
#             # Use regular system
#             valid_reference, ref_index = self.find_valid_reference(current_pos)


#         self.publish_reference_zones(ref_index)

#         if ref_index != -1:
#             self.last_used_ref_index = ref_index


#         # Compute distance from the active reference
#         if valid_reference is not None and ref_index != -1:
#             ref_pos = self.reference_scans[ref_index][1]
#             dx = current_pos[0] - ref_pos[0]
#             dy = current_pos[1] - ref_pos[1]
#             distance = (dx**2 + dy**2)**0.5

#             # Interpolate fitness threshold based on distance
#             if distance <= 1.0:
#                 threshold_fitness = 0.6 - 0.15 * (distance / 1.0)  # From 0.6 to 0.45
#             elif distance <= self.reference_radius:
#                 threshold_fitness = 0.45 - 0.15 * ((distance - 1.0) / (self.reference_radius - 1.0))  # From 0.45 to 0.3
#             else:
#                 threshold_fitness = 0.3  # Max looseness at edge or beyond
#         else:
#             threshold_fitness = 0.6  # Default for new reference    


#         # If no valid reference is available for the current position,
#         # then create a new reference scan.
#         if valid_reference is None:
#             # Transform scan to odom frame
#             if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
#                 tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#                 T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)
#             else:
#                 self.get_logger().warn("TF odom-lidar lookup failed for mini-ICP; skipping update")
#                 self.publish_icp_transform(self.latest_correction, msg)
#                 return

#             new_scan.transform(T_odom_lidar)

#             # Try Mini-ICP
#             use_as_reference = False
#             closest_ref = None
#             if self.use_special_refs:
#                 if 0 <= self.last_used_ref_index < len(self.special_references):
#                     closest_ref = self.special_references[self.last_used_ref_index][0]
#             else:

#                 if 0 <= self.last_used_ref_index < len(self.reference_scans):
#                     closest_ref = self.reference_scans[self.last_used_ref_index][0]

#             if closest_ref is not None:
#                 threshold = 0.12
#                 icp_mini_result = o3d.pipelines.registration.registration_icp(
#                     new_scan, closest_ref, threshold, self.latest_correction,
#                     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25000)
#                 )
#                 self.get_logger().info(f"[MiniICP] Fitness: {icp_mini_result.fitness:.3f} | RMSE: {icp_mini_result.inlier_rmse:.3f}")
#                 if icp_mini_result.fitness > 0.2:
#                     self.get_logger().info("Mini-ICP applied to refine new reference scan.")
#                     self.latest_correction = icp_mini_result.transformation
#                     use_as_reference = True
#                 else:
#                     self.get_logger().warn("Mini-ICP rejected due to low fitness.")
#                     pass
#             else:
#                 # No previous reference available for refinement, accept this one directly
#                 self.get_logger().info("No prior reference — accepting new scan as initial reference.")
#                 use_as_reference = True

#             if use_as_reference:
#                 # Apply correction and store
#                 T_map_odom = self.latest_correction
#                 new_scan.transform(T_map_odom)

#                 if self.use_special_refs:
#                     self.special_references.append((new_scan, current_pos))
#                     self.get_logger().info("Created new special reference")
#                 else:
#                     self.reference_scans.append((new_scan, current_pos))
#                     self.get_logger().info("Created new regular reference")
#                 valid_reference = new_scan
#                 ref_msg = open3d_to_pointcloud2(valid_reference)
#                 self.ref_scan_pub.publish(ref_msg)
#             else:
#                 self.get_logger().warn("New scan rejected — not stored as reference.")
#                 pass

#             return


        
#         if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
#             tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#             # print(tf_map_lidar)
#             T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)

#         else:
#             self.get_logger().warn("TF odom-lidar Lookup failed: skipping update")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return
        
#         # Transform new scan to the map frame
#         new_scan.transform(T_odom_lidar)
#         # if self.count == 10:
#         new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(0, 255, 0))  # green
#         self.new_scan_pub.publish(new_msg)


#         # ICP Convergence threshold
#         threshold = 0.12
#         # Run ICP: Align `new_scan` (source) to `accumulated_map` (target)
#         icp_result = o3d.pipelines.registration.registration_icp(
#             new_scan, valid_reference, threshold, self.latest_correction,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=12000))
        

#         #consider using fitness to choose activ/new reference frame
#         self.get_logger().info(f'inlier: {icp_result.inlier_rmse}, fitness: {icp_result.fitness}',)
#         if icp_result.inlier_rmse > 0.07 or icp_result.fitness < threshold_fitness:
#             self.get_logger().warn("ICP registration did not match well, skipping update.")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return

#         # Update the correction transform
#         self.latest_correction = icp_result.transformation
       
        
#         self.icp_publish_counter += 1.0
#         if self.icp_publish_counter % 1.0 == 0.0:
#             self.get_logger().info("uppdate")
#             self.publish_icp_transform(self.latest_correction, msg)

#             #consider replacing this with the transform from the icp directly after debugging
#             if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(seconds=0)):
#                 tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
#                 T_map_odom = self.transform_to_matrix(tf_map_odom)

#             else:
#                 self.get_logger().warn("TF map-odom Lookup failed: skipping update")
#                 self.icp_publish_counter += 1.0
#                 if self.icp_publish_counter % 1.0 == 0.0:
#                     self.publish_icp_transform(self.latest_correction, msg)
#                 return
#             new_scan.transform(T_map_odom)

#             self.count = self.count + 1
#             new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(255, 0, 0))  #red
#             self.new_scan_icp_pub.publish(new_msg)
#             self.merged_pc = None
#             self.pc_counter = 0

#         # Always republish the first scan
#         if self.first_scan is not None:
#             first_msg = open3d_to_colored_pointcloud2(self.first_scan, rgb_color=(0, 0, 255))  # Blue
#             self.first_scan_pub.publish(first_msg)


#     def omega_callback(self, msg):
#         self.omega = msg.data

#     def find_valid_reference(self, current_pos):
#         for i, (ref_pc, ref_pos) in enumerate(self.reference_scans):
#             dx = current_pos[0] - ref_pos[0]
#             dy = current_pos[1] - ref_pos[1]
#             if (dx**2 + dy**2)**0.5 < self.reference_radius:
#                 return ref_pc, i  # Return the point cloud and index
#         return None, -1




#     def transform_to_matrix(self, tf_msg):
#         """Converts a ROS2 TransformStamped to a 4x4 transformation matrix."""
#         trans = tf_msg.transform.translation
#         rot = tf_msg.transform.rotation

#         # Convert quaternion to a 3x3 rotation matrix
#         r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

#         # Construct the 4x4 transformation matrix
#         matrix = np.eye(4)
#         matrix[:3, :3] = r
#         matrix[:3, 3] = [trans.x, trans.y, trans.z]

#         return matrix


#     def publish_reference_zones(self, active_index):
#         for i, (_, ref_pos) in enumerate(self.reference_scans):
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "reference_zones"
#             marker.id = i
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
#             marker.pose.position.x = ref_pos[0]
#             marker.pose.position.y = ref_pos[1]
#             marker.pose.position.z = 0.0  # 2D

#             marker.pose.orientation.w = 1.0  # No rotation

#             marker.scale.x = self.reference_radius * 2.0
#             marker.scale.y = self.reference_radius * 2.0
#             marker.scale.z = 0.01  # Thin cylinder

#             if i == active_index:
#                 # Active zone — green
#                 marker.color.r = 0.0
#                 marker.color.g = 1.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.5
#             else:
#                 # Inactive zone — red
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.2

#             marker.lifetime.sec = 0  # Keep until overwritten
#             self.marker_pub.publish(marker)

#         # Special reference zones
#         for i, (_, ref_pos) in enumerate(self.special_references):
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "special_reference_zones"
#             marker.id = 1000 + i  # avoid ID conflict
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
#             marker.pose.position.x = ref_pos[0]
#             marker.pose.position.y = ref_pos[1]
#             marker.pose.position.z = 0.0
#             marker.pose.orientation.w = 1.0
#             marker.scale.x = self.special_ref_radius * 2.0
#             marker.scale.y = self.special_ref_radius * 2.0
#             marker.scale.z = 0.01
#             if i == active_index and self.use_special_refs:
#                 marker.color.r = 0.0
#                 marker.color.g = 1.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.5
#             else:
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.2
#             marker.lifetime.sec = 0
#             self.marker_pub.publish(marker)


#     def publish_icp_transform(self, transform, msg):
#         """Publishes ICP correction directly to /tf."""
#         # get latest odom transform

#         odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0))
    
#         t = TransformStamped()
#         t.header.stamp = odom_transform.header.stamp
#         t.header.frame_id = 'map'
#         t.child_frame_id = 'odom'

#         t.transform.translation.x = transform[0, 3]
#         t.transform.translation.y = transform[1, 3]
#         t.transform.translation.z = 0.0  # 2D

#         r = R.from_matrix(transform[0:3, 0:3])
#         quat = r.as_quat()
#         t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat

#         # **Broadcast the transform to /tf**
#         self.tf_broadcaster.sendTransform(t)

#         self.get_logger().info("Published ICP correction to /tf.")

# def main():
#     rclpy.init()
#     node = IcpMappingNode()
#     ex = MultiThreadedExecutor()
#     ex.add_node(node)
#     ex.spin()
#     # rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

















# import sys
# print("Running with Python:", sys.executable)
# import numpy as np
# import open3d as o3d
# import rclpy
# from rclpy.node import Node
# import tf2_ros
# from geometry_msgs.msg import TransformStamped
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import LaserScan
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from scipy.spatial import cKDTree
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Header
# import sensor_msgs_py.point_cloud2 as pc2
# from sensor_msgs.msg import PointField
# from std_msgs.msg import Float32
# from visualization_msgs.msg import Marker
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist



# # Convert LiDAR scan to Open3D point cloud
# def convert_scan_to_open3d(scan_msg):
#     """Converts a ROS2 LaserScan message into an Open3D point cloud."""
#     # Get all valid range values and corresponding angles
#     ranges = np.array(scan_msg.ranges)
#     angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

#     valid = np.isfinite(ranges)
#     ranges = ranges[valid]
#     angles = angles[valid]

#     # Convert to Cartesian coordinates
#     x = ranges * np.cos(angles)
#     y = ranges * np.sin(angles)
#     points = np.column_stack((x, y))

#     # --- Region filter parameters ---
#     a = 0.085
#     b = 0.12
#     r = 0.39
#     q = 0.31

#     # Apply region mask
#     x_scaled = (points[:, 0] + a) / r
#     y_scaled = (points[:, 1] + b) / q
#     mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) >= 1
#     filtered_points = points[mask]

#     # KDTree-based neighbor filtering
#     radius = 0.01  # 1 cm neighborhood radius
#     kdtree = cKDTree(filtered_points)
#     to_keep = np.ones(filtered_points.shape[0], dtype=bool)

#     for i, p in enumerate(filtered_points):
#         if not to_keep[i]:
#             continue  # Already marked for removal
#         idxs = kdtree.query_ball_point(p, r=radius)
#         idxs.remove(i)  # Don't remove itself
#         to_keep[idxs] = False  # Remove all close neighbors

#     final_points = filtered_points#[to_keep]
#     pointcloud_xyz = np.column_stack((final_points, np.zeros(len(final_points))))
#     # Convert to Open3D point cloud
#     pc = o3d.geometry.PointCloud()
#     pc.points = o3d.utility.Vector3dVector(pointcloud_xyz)
#     return pc

# def open3d_to_pointcloud2(o3d_pc, frame_id="map"):
#     """Convert Open3D point cloud to ROS2 PointCloud2 message."""
#     points = np.asarray(o3d_pc.points)

#     header = Header()
#     header.stamp = rclpy.time.Time().to_msg()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#     ]

#     pc2_msg = pc2.create_cloud(header, fields, points)
#     return pc2_msg

# def open3d_to_colored_pointcloud2(o3d_pc, rgb_color=(255, 255, 255), frame_id="map"):
#     """Convert Open3D point cloud to a colored PointCloud2 message."""
#     points = np.asarray(o3d_pc.points)
#     r, g, b = rgb_color

#     # Encode RGB as packed float
#     rgb_floats = np.array([
#         np.frombuffer(np.uint32((r << 16) | (g << 8) | b).tobytes(), dtype=np.float32)[0]
#         for _ in range(len(points))
#     ])

#     # Combine (x, y, z, rgb)
#     colored_points = np.hstack((points, rgb_floats.reshape(-1, 1)))

#     header = Header()
#     header.stamp = rclpy.time.Time().to_msg()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
#     ]

#     return pc2.create_cloud(header, fields, colored_points)



# class IcpMappingNode(Node):
#     def __init__(self):
#         super().__init__('icp_mapping_node')
#         print("yassir mapping")
#         group = ReentrantCallbackGroup()
#         # Subscribe to the LiDAR scan topic
#         self.subscription = self.create_subscription(
#             LaserScan, '/scan', self.scan_callback, 1, callback_group=group)
        
#         # Subscribe to /cmd_vel
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10, callback_group=group)

#         # Subscribe to angular velocity
#         self.create_subscription(Float32, '/angular_velocity', self.omega_callback, 10, callback_group=group)


#         self.first_scan_pub = self.create_publisher(PointCloud2, '/first_scan', 10)
#         self.ref_scan_pub = self.create_publisher(PointCloud2, '/reference_scan', 10)
#         self.new_scan_pub = self.create_publisher(PointCloud2, '/new_scan', 10)
#         self.new_scan_icp_pub = self.create_publisher(PointCloud2, '/new_scan_icp', 10)

#         self.marker_pub = self.create_publisher(Marker, '/reference_zones', 10)

#         # Publish to boolean topic
#         self.waiting_pub = self.create_publisher(Bool, '/should_stop', 10)


#         # **TF Broadcaster instead of topic publisher**
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # TF buffer and listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

#         # Store the latest correction transform (initialize as identity)
#         self.latest_correction = None

#         self.accumulated_map = None  # Store the global map in `map` frame

#         self.reference_scan = None
#         self.count = 0
#         self.icp_publish_counter = 0
#         self.scan_counter = 0
#         self.pc_counter = 0
#         self.pc_buffer_size = 10
#         self.merged_pc = None
#         self.bad_match_counter = 0
#         self.first_scan = None
#         self.omega = 0.0  # Latest angular velocity
#         self.omega_threshold = 0.2  # rad/s
#         self.reference_scans = []  # List of (pointcloud, (x, y)) s
#         self.reference_radius = 2.5  # meters
#         self.last_used_ref_index = -1


#         # Special reference region config (x in meters)
#         self.special_ref_zone_xmin = 7.45
#         self.special_ref_zone_xmax = 8.25
#         self.special_ref_radius = 0.70 
#         self.special_references = []  # list of (pointcloud, (x, y))
#         self.use_special_refs = False
#         # Store latest cmd_vel
#         self.latest_linear = (0.0, 0.0)
#         self.latest_angular = 0.0
#         # Thresholds for stillness detection
#         self.still_threshold_lin = 0.01  # m/s
#         self.still_threshold_ang = 0.01  # rad/s

#         self.waiting_for_stillness = False



#     def scan_callback(self, msg):
#         """Transforms new scan to map frame, performs ICP, and updates transform."""
#         self.scan_counter += 1.0

#         if self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time(seconds=0)):
#             self.get_logger().info("odom ready, icp intiated")

#         else:
#             self.get_logger().warn("TF odom-base_link lookup failed for first scan; skipping scan, if you see this at the start=ok, otherwise problem")
#             return

#         if self.latest_correction is None:
#             self.latest_correction = np.eye(4)
#             self.publish_icp_transform(self.latest_correction, msg)
#             return
        
#         if self.first_scan is None:
#             # Transform the first scan to the map frame
#             if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
#                 tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
#                 T_map_lidar = self.transform_to_matrix(tf_map_lidar)
#                 first_scan = convert_scan_to_open3d(msg)
#                 first_scan.transform(T_map_lidar)
#                 self.first_scan = first_scan
#             else:
#                 self.get_logger().warn("TF map-lidar lookup failed for first scan; skipping setting it")
#                 pass

        
#         if abs(self.omega) > self.omega_threshold:
#             self.get_logger().info(f"Skipping ICP — omega too high: {self.omega:.2f} rad/s")
#             self.publish_icp_transform(self.latest_correction, msg)
#             return


#         new_scan = convert_scan_to_open3d(msg)

#         if self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time(seconds=0)):
#             transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(seconds=0))
#             current_x = transform.transform.translation.x
#             current_y = transform.transform.translation.y
#         else:
#             self.get_logger().warn("TF map-base_link Lookup failed: skipping update")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return

#         self.use_special_refs = self.special_ref_zone_xmin <= current_x <= self.special_ref_zone_xmax
#         current_pos = (current_x, current_y)

#         if self.use_special_refs:
#             # Try to find a valid special reference
#             valid_reference = None
#             ref_index = -1
#             for i, (ref_pc, ref_pos) in enumerate(self.special_references):
#                 dist = np.linalg.norm(np.array(current_pos) - np.array(ref_pos))
#                 if dist <= self.special_ref_radius:
#                     valid_reference = ref_pc
#                     ref_index = i  # optional: use for visualization
#                     break  # stop at the first valid one

#         else:
#             # Use regular system
#             valid_reference, ref_index = self.find_valid_reference(current_pos)

#         self.publish_reference_zones(ref_index)

#         if ref_index != -1:
#             self.last_used_ref_index = ref_index

#         dist_to_closest_keyframe = 0.0 
#         # Compute distance from the active reference
#         if valid_reference is not None and ref_index != -1:
#             ref_pos = self.reference_scans[ref_index][1]
#             dx = current_pos[0] - ref_pos[0]
#             dy = current_pos[1] - ref_pos[1]
#             distance = (dx**2 + dy**2)**0.5
#             dist_to_closest_keyframe = distance

#             # Interpolate fitness threshold based on distance
#             if distance <= 1.0:
#                 threshold_fitness = 0.6 - 0.15 * (distance / 1.0)  # From 0.6 to 0.45
#             elif distance <= self.reference_radius:
#                 threshold_fitness = 0.45 - 0.15 * ((distance - 1.0) / (self.reference_radius - 1.0))  # From 0.45 to 0.3
#             else:
#                 threshold_fitness = 0.3  # Max looseness at edge or beyond
#         else:
#             threshold_fitness = 0.6  # Default for new reference    

#         # If no valid reference is available for the current position,
#         # then create a new reference scan.
#         if valid_reference is None:
#             valid_reference = self.handle_new_reference(new_scan, current_pos, msg)
#         if dist_to_closest_keyframe > 1.5: 
#             valid_reference = self.handle_new_reference(new_scan, current_pos, msg)

#         if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
#             tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#             T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)

#         else:
#             self.get_logger().warn("TF odom-lidar Lookup failed: skipping update")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return
        
#         # Transform new scan to the map frame
#         new_scan.transform(T_odom_lidar)
#         # if self.count == 10:
#         new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(0, 255, 0))  # green
#         self.new_scan_pub.publish(new_msg)

#         # ICP Convergence threshold
#         threshold = 0.12
#         # Run ICP: Align `new_scan` (source) to `accumulated_map` (target)
#         if valid_reference is None:
#             return
#         icp_result = o3d.pipelines.registration.registration_icp(
#             new_scan, valid_reference, threshold, self.latest_correction,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=12000))
        

#         #consider using fitness to choose activ/new reference frame
#         self.get_logger().info(f'inlier: {icp_result.inlier_rmse}, fitness: {icp_result.fitness}',)
#         if icp_result.inlier_rmse > 0.07 or icp_result.fitness < threshold_fitness:
#             self.get_logger().warn("ICP registration did not match well, skipping update.")
#             self.icp_publish_counter += 1.0
#             if self.icp_publish_counter % 1.0 == 0.0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return

#         # Update the correction transform
#         self.latest_correction = icp_result.transformation
       
        
#         self.icp_publish_counter += 1.0
#         if self.icp_publish_counter % 1.0 == 0.0:
#             self.get_logger().info("uppdate")
#             self.publish_icp_transform(self.latest_correction, msg)

#             #consider replacing this with the transform from the icp directly after debugging
#             if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(seconds=0)):
#                 tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
#                 T_map_odom = self.transform_to_matrix(tf_map_odom)

#             else:
#                 self.get_logger().warn("TF map-odom Lookup failed: skipping update")
#                 self.icp_publish_counter += 1.0
#                 if self.icp_publish_counter % 1.0 == 0.0:
#                     self.publish_icp_transform(self.latest_correction, msg)
#                 return
#             new_scan.transform(T_map_odom)

#             self.count = self.count + 1
#             new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(255, 0, 0))  #red
#             self.new_scan_icp_pub.publish(new_msg)
#             self.merged_pc = None
#             self.pc_counter = 0

#         # Always republish the first scan
#         if self.first_scan is not None:
#             first_msg = open3d_to_colored_pointcloud2(self.first_scan, rgb_color=(0, 0, 255))  # Blue
#             self.first_scan_pub.publish(first_msg)

#     def handle_new_reference(self, new_scan, current_pos, msg):
#         # If not already in waiting mode, enter it
#         if not self.waiting_for_stillness:
#             self.waiting_for_stillness = True
#             self.waiting_pub.publish(Bool(data=True))

#         # Check if robot is still
#         if not self.is_robot_still():
#             # Keep using the last valid reference (regardless of pose)
#             valid_reference = self.get_last_reference()
#             if valid_reference is None:
#                 self.publish_icp_transform(self.latest_correction, msg)
#                 self.get_logger().warn("fastnar här")
#                 return

#             # Continue with alignment using last reference
#             # (skip rest of this block)
#             self.get_logger().warn("fastnar här 1")
#             return valid_reference
#         else:
#             # Robot is now still → time to create a reference
#             self.waiting_pub.publish(Bool(data=False))
#             self.waiting_for_stillness = False


#             # Transform scan to odom frame
#             if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
#                 tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#                 T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)
#             else:
#                 self.get_logger().warn("TF odom-lidar lookup failed for mini-ICP; skipping update")
#                 self.publish_icp_transform(self.latest_correction, msg)
#                 return

#             new_scan.transform(T_odom_lidar)

#             # Try Mini-ICP
#             use_as_reference = False

#             closest_ref = self.get_last_reference()

#             if closest_ref is not None:
#                 threshold = 0.12
#                 icp_mini_result = o3d.pipelines.registration.registration_icp(
#                     new_scan, closest_ref, threshold, self.latest_correction,
#                     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25000)
#                 )
#                 self.get_logger().info(f"[MiniICP] Fitness: {icp_mini_result.fitness:.3f} | RMSE: {icp_mini_result.inlier_rmse:.3f}")
#                 if icp_mini_result.fitness > 0.1:
#                     self.get_logger().info("Mini-ICP applied to refine new reference scan.")
#                     self.latest_correction = icp_mini_result.transformation
#                     use_as_reference = True
#                 else:
#                     self.get_logger().warn("Mini-ICP rejected due to low fitness.")
#                     pass
#             else:
#                 # No previous reference available, accept directly
#                 self.get_logger().info("No prior reference — accepting new scan as initial reference.")
#                 use_as_reference = True

#             if use_as_reference:
#                 # Apply correction and store
#                 T_map_odom = self.latest_correction
#                 new_scan.transform(T_map_odom)

#                 if self.use_special_refs:
#                     self.special_references.append((new_scan, current_pos))
#                     self.get_logger().info("Created new special reference")
#                 else:
#                     self.reference_scans.append((new_scan, current_pos))
#                     self.get_logger().info("Created new regular reference")
#                 valid_reference = new_scan
#                 ref_msg = open3d_to_pointcloud2(valid_reference)
#                 self.ref_scan_pub.publish(ref_msg)
#             else:
#                 # Even if rejected, clear waiting state
#                 self.waiting_for_stillness = False
#                 self.waiting_pub.publish(Bool(data=False))
#                 self.get_logger().warn("New scan rejected — not stored as reference.")
#                 pass

#     def cmd_vel_callback(self, msg):
#         self.latest_linear = (msg.linear.x, msg.linear.y)
#         self.latest_angular = msg.angular.z

    
#     def is_robot_still(self):
#         vx, vy = self.latest_linear
#         w = self.latest_angular
#         return abs(vx) < self.still_threshold_lin and abs(vy) < self.still_threshold_lin and abs(w) < self.still_threshold_ang

#     def get_last_reference(self):
#         if self.use_special_refs and 0 <= self.last_used_ref_index < len(self.special_references):
#             return self.special_references[self.last_used_ref_index][0]
#         elif 0 <= self.last_used_ref_index < len(self.reference_scans):
#             return self.reference_scans[self.last_used_ref_index][0]
#         else:
#             return None

#     def omega_callback(self, msg):
#         self.omega = msg.data

#     def find_valid_reference(self, current_pos):
#         for i, (ref_pc, ref_pos) in enumerate(self.reference_scans):
#             dx = current_pos[0] - ref_pos[0]
#             dy = current_pos[1] - ref_pos[1]
#             if (dx**2 + dy**2)**0.5 < self.reference_radius:
#                 return ref_pc, i  # Return the point cloud and index
#         return None, -1




#     def transform_to_matrix(self, tf_msg):
#         """Converts a ROS2 TransformStamped to a 4x4 transformation matrix."""
#         trans = tf_msg.transform.translation
#         rot = tf_msg.transform.rotation

#         # Convert quaternion to a 3x3 rotation matrix
#         r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

#         # Construct the 4x4 transformation matrix
#         matrix = np.eye(4)
#         matrix[:3, :3] = r
#         matrix[:3, 3] = [trans.x, trans.y, trans.z]

#         return matrix


#     def publish_reference_zones(self, active_index):
#         for i, (_, ref_pos) in enumerate(self.reference_scans):
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "reference_zones"
#             marker.id = i
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
#             marker.pose.position.x = ref_pos[0]
#             marker.pose.position.y = ref_pos[1]
#             marker.pose.position.z = 0.0  # 2D

#             marker.pose.orientation.w = 1.0  # No rotation

#             marker.scale.x = self.reference_radius * 2.0
#             marker.scale.y = self.reference_radius * 2.0
#             marker.scale.z = 0.01  # Thin cylinder

#             if i == active_index:
#                 # Active zone — green
#                 marker.color.r = 0.0
#                 marker.color.g = 1.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.5
#             else:
#                 # Inactive zone — red
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.2

#             marker.lifetime.sec = 0  # Keep until overwritten
#             self.marker_pub.publish(marker)

#         # Special reference zones
#         for i, (_, ref_pos) in enumerate(self.special_references):
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "special_reference_zones"
#             marker.id = 1000 + i  # avoid ID conflict
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
#             marker.pose.position.x = ref_pos[0]
#             marker.pose.position.y = ref_pos[1]
#             marker.pose.position.z = 0.0
#             marker.pose.orientation.w = 1.0
#             marker.scale.x = self.special_ref_radius * 2.0
#             marker.scale.y = self.special_ref_radius * 2.0
#             marker.scale.z = 0.01
#             if i == active_index and self.use_special_refs:
#                 marker.color.r = 0.0
#                 marker.color.g = 1.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.5
#             else:
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#                 marker.color.a = 0.2
#             marker.lifetime.sec = 0
#             self.marker_pub.publish(marker)


#     def publish_icp_transform(self, transform, msg):
#         """Publishes ICP correction directly to /tf."""
#         # get latest odom transform

#         odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0))
    
#         t = TransformStamped()
#         t.header.stamp = odom_transform.header.stamp
#         t.header.frame_id = 'map'
#         t.child_frame_id = 'odom'

#         t.transform.translation.x = transform[0, 3]
#         t.transform.translation.y = transform[1, 3]
#         t.transform.translation.z = 0.0  # 2D

#         r = R.from_matrix(transform[0:3, 0:3])
#         quat = r.as_quat()
#         t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat

#         # **Broadcast the transform to /tf**
#         self.tf_broadcaster.sendTransform(t)

#         self.get_logger().info("Published ICP correction to /tf.")

# def main():
#     rclpy.init()
#     node = IcpMappingNode()
#     ex = MultiThreadedExecutor()
#     ex.add_node(node)
#     ex.spin()
#     # rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()













import sys
print("Running with Python:", sys.executable)
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist



# Convert LiDAR scan to Open3D point cloud
def convert_scan_to_open3d(scan_msg):
    """Converts a ROS2 LaserScan message into an Open3D point cloud."""
    # Get all valid range values and corresponding angles
    ranges = np.array(scan_msg.ranges)
    angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

    valid = np.isfinite(ranges)
    ranges = ranges[valid]
    angles = angles[valid]

    # Convert to Cartesian coordinates
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    points = np.column_stack((x, y))

    # --- Region filter parameters ---
    a = 0.085
    b = 0.12
    r = 0.39
    q = 0.31

    # Apply region mask
    x_scaled = (points[:, 0] + a) / r
    y_scaled = (points[:, 1] + b) / q
    mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) >= 1
    filtered_points = points[mask]

    # KDTree-based neighbor filtering
    radius = 0.01  # 1 cm neighborhood radius
    kdtree = cKDTree(filtered_points)
    to_keep = np.ones(filtered_points.shape[0], dtype=bool)

    for i, p in enumerate(filtered_points):
        if not to_keep[i]:
            continue  # Already marked for removal
        idxs = kdtree.query_ball_point(p, r=radius)
        idxs.remove(i)  # Don't remove itself
        to_keep[idxs] = False  # Remove all close neighbors

    final_points = filtered_points#[to_keep]
    pointcloud_xyz = np.column_stack((final_points, np.zeros(len(final_points))))
    # Convert to Open3D point cloud
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pointcloud_xyz)
    return pc

def open3d_to_pointcloud2(o3d_pc, frame_id="map"):
    """Convert Open3D point cloud to ROS2 PointCloud2 message."""
    points = np.asarray(o3d_pc.points)

    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    pc2_msg = pc2.create_cloud(header, fields, points)
    return pc2_msg

def open3d_to_colored_pointcloud2(o3d_pc, rgb_color=(255, 255, 255), frame_id="map"):
    """Convert Open3D point cloud to a colored PointCloud2 message."""
    points = np.asarray(o3d_pc.points)
    r, g, b = rgb_color

    # Encode RGB as packed float
    rgb_floats = np.array([
        np.frombuffer(np.uint32((r << 16) | (g << 8) | b).tobytes(), dtype=np.float32)[0]
        for _ in range(len(points))
    ])

    # Combine (x, y, z, rgb)
    colored_points = np.hstack((points, rgb_floats.reshape(-1, 1)))

    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    return pc2.create_cloud(header, fields, colored_points)



class IcpMappingNode(Node):
    def __init__(self):
        super().__init__('icp_mapping_node')
        print("yassir mapping")
        group = ReentrantCallbackGroup()
        # Subscribe to the LiDAR scan topic
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 1, callback_group=group)
        
        # Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10, callback_group=group)

        
        # Subscribe to angular velocity
        self.create_subscription(Float32, '/angular_velocity', self.omega_callback, 10, callback_group=group)


        self.first_scan_pub = self.create_publisher(PointCloud2, '/first_scan', 10)
        self.ref_scan_pub = self.create_publisher(PointCloud2, '/reference_scan', 10)
        self.new_scan_pub = self.create_publisher(PointCloud2, '/new_scan', 10)
        self.new_scan_icp_pub = self.create_publisher(PointCloud2, '/new_scan_icp', 10)

        self.marker_pub = self.create_publisher(Marker, '/reference_zones', 10)

        # Publish to boolean topic
        self.waiting_pub = self.create_publisher(Bool, '/icp_waiting_for_stillness', 10)


        # **TF Broadcaster instead of topic publisher**
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        # Store the latest correction transform (initialize as identity)
        self.latest_correction = None

        self.accumulated_map = None  # Store the global map in map frame

        self.reference_scan = None
        self.count = 0
        self.icp_publish_counter = 0
        self.scan_counter = 0
        self.pc_counter = 0
        self.pc_buffer_size = 10
        self.merged_pc = None
        self.bad_match_counter = 0
        self.first_scan = None
        self.omega = 0.0  # Latest angular velocity
        self.omega_threshold = 0.2  # rad/s
        self.reference_scans = []  # List of (pointcloud, (x, y)) s
        self.reference_radius = 2.5  # meters
        self.last_used_ref_index = -1


        # Special reference region config (x in meters)
        # self.special_ref_zone_xmin = 1
        # self.special_ref_zone_xmax = 2.5
        self.special_ref_zone_xmin = 7.45
        self.special_ref_zone_xmax = 8.25
        self.special_ref_radius = 0.70 
        self.special_references = []  # list of (pointcloud, (x, y))
        self.use_special_refs = False
        self.last_ref_from_special = False  # Track which list the index belongs to

        # Store latest cmd_vel
        self.latest_linear = (0.0, 0.0)
        self.latest_angular = 0.0
        # Thresholds for stillness detection
        self.still_threshold_lin = 0.01  # m/s
        self.still_threshold_ang = 0.01  # rad/s

        self.waiting_for_stillness = False




    def scan_callback(self, msg):
        """Transforms new scan to map frame, performs ICP, and updates transform."""
        self.scan_counter += 1.0

        if self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time(seconds=0)):
            # self.get_logger().info("odom ready, icp intiated")
            pass

        else:
            # self.get_logger().warn("TF odom-base_link lookup failed for first scan; skipping scan, if you see this at the start=ok, otherwise problem")
            return

        if self.latest_correction is None:
            self.latest_correction = np.eye(4)
            self.publish_icp_transform(self.latest_correction, msg)
            return
        
        if self.first_scan is None:
            # Transform the first scan to the map frame
            if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
                tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
                T_map_lidar = self.transform_to_matrix(tf_map_lidar)
                first_scan = convert_scan_to_open3d(msg)
                first_scan.transform(T_map_lidar)
                self.first_scan = first_scan
            else:
                # self.get_logger().warn("TF map-lidar lookup failed for first scan; skipping setting it")
                pass

        
        if abs(self.omega) > self.omega_threshold:
            # self.get_logger().info(f"Skipping ICP — omega too high: {self.omega:.2f} rad/s")
            self.publish_icp_transform(self.latest_correction, msg)
            return


        new_scan = convert_scan_to_open3d(msg)

        if self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time(seconds=0)):
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(seconds=0))
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
        else:
            # self.get_logger().warn("TF odom-base_link Lookup failed: skipping update")
            self.icp_publish_counter += 1.0
            if self.icp_publish_counter % 1.0 == 0.0:
                self.publish_icp_transform(self.latest_correction, msg)
            return

        self.use_special_refs = self.special_ref_zone_xmin <= current_x <= self.special_ref_zone_xmax
        current_pos = (current_x, current_y)

        if self.use_special_refs:
            # Try to find a valid special reference
            valid_reference = None
            ref_index = -1
            for i, (ref_pc, ref_pos) in enumerate(self.special_references):
                dist = np.linalg.norm(np.array(current_pos) - np.array(ref_pos))
                if dist <= self.special_ref_radius:
                    valid_reference = ref_pc
                    ref_index = i  # optional: use for visualization
                    break  # stop at the first valid one

        else:
            # Use regular system
            valid_reference, ref_index = self.find_valid_reference(current_pos)


        # If no valid reference is available for the current position,
        # then create a new reference scan.
        if valid_reference is None:
            # If not already in waiting mode, enter it
            if not self.waiting_for_stillness:
                self.waiting_for_stillness = True
                self.waiting_pub.publish(Bool(data=True))

            # Check if robot is still
            if not self.is_robot_still():
                # Keep using the last valid reference (regardless of pose)
                valid_reference = self.get_last_reference()
                ref_index = self.last_used_ref_index
                if valid_reference is None:
                    self.publish_icp_transform(self.latest_correction, msg)
                    return
                
                # Continue with alignment using last reference
            else:
                # Robot is now still → time to create a reference
                self.waiting_pub.publish(Bool(data=False))
                self.waiting_for_stillness = False


                # Transform scan to odom frame
                if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
                    tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
                    T_map_lidar = self.transform_to_matrix(tf_map_lidar)
                else:
                    # # self.get_logger().warn("TF odom-lidar lookup failed for mini-ICP; skipping update")
                    self.publish_icp_transform(self.latest_correction, msg)
                    return

                new_scan.transform(T_map_lidar)

                # Try Mini-ICP
                use_as_reference = False
                closest_ref = self.get_last_reference()
                if closest_ref is not None:
                    threshold = 0.12
                    icp_mini_result = o3d.pipelines.registration.registration_icp(
                        new_scan, closest_ref, threshold, self.latest_correction,
                        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25000)
                    )
                    self.get_logger().info(f"[MiniICP] Fitness: {icp_mini_result.fitness:.3f} | RMSE: {icp_mini_result.inlier_rmse:.3f}")
                    if icp_mini_result.fitness > 0.6:
                    # if icp_mini_result.inlier_rmse < 0.17 and  icp_mini_result.fitness > 0.6:

                        # self.get_logger().info("Mini-ICP applied to refine new reference scan.")
                        self.latest_correction = icp_mini_result.transformation
                        use_as_reference = True
                    else:
                        self.get_logger().warn("Mini-ICP rejected due to low fitness.")
                        pass
                else:
                    # No previous reference available for refinement, accept this one directly
                    # self.get_logger().info("No prior reference — accepting new scan as initial reference.")
                    use_as_reference = True

                if use_as_reference:
                    # Apply correction and store
                    T_map_odom = self.latest_correction
                    new_scan.transform(T_map_odom)

                    if self.use_special_refs:
                        self.special_references.append((new_scan, current_pos))
                        # self.get_logger().info("Created new special reference")
                        self.last_used_ref_index = len(self.special_references) - 1
                        self.last_ref_from_special = True
                    else:
                        self.reference_scans.append((new_scan, current_pos))
                        # self.get_logger().info("Created new regular reference")
                        self.last_used_ref_index = len(self.reference_scans) - 1
                        self.last_ref_from_special = False
                    valid_reference = new_scan
                    ref_msg = open3d_to_pointcloud2(valid_reference)
                    self.ref_scan_pub.publish(ref_msg)
                else:
                    # Even if rejected, clear waiting state
                    self.waiting_for_stillness = False
                    self.waiting_pub.publish(Bool(data=False))
                    # self.get_logger().warn("New scan rejected — not stored as reference.")
                    pass

                return

        self.publish_reference_zones(ref_index)
         # Compute distance from the active reference
        if valid_reference is not None and ref_index != -1:
            if self.waiting_for_stillness and self.last_ref_from_special:
                ref_pos = self.special_references[ref_index][1]
            elif self.waiting_for_stillness and not self.last_ref_from_special:
                ref_pos = self.reference_scans[ref_index][1]
            elif self.use_special_refs:
                ref_pos = self.special_references[ref_index][1]
            else:
                ref_pos = self.reference_scans[ref_index][1]
            dx = current_pos[0] - ref_pos[0]
            dy = current_pos[1] - ref_pos[1]
            distance = (dx**2 + dy**2)**0.5

            # Interpolate fitness threshold based on distance
            if distance <= 1.0:
                threshold_fitness = 0.6 - 0.15 * (distance / 1.0)  # From 0.6 to 0.45
            elif distance <= self.reference_radius:
                threshold_fitness = 0.45 - 0.15 * ((distance - 1.0) / (self.reference_radius - 1.0))  # From 0.45 to 0.3
            else:
                threshold_fitness = 0.3  # Max looseness at edge or beyond
        elif self.waiting_for_stillness:
            threshold_fitness = 0.3
        else:
            threshold_fitness = 0.6  # Default for new reference    

        
        if self.tf_buffer.can_transform("odom", "lidar_link", rclpy.time.Time(seconds=0)):
            tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
            # print(tf_map_lidar)
            ## self.get_logger().info(tf_odom_lidar)
            T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)
            ## self.get_logger().info(tf_odom_lidar)

        else:
            # self.get_logger().warn("TF odom-lidar Lookup failed: skipping update")
            self.icp_publish_counter += 1.0
            if self.icp_publish_counter % 1.0 == 0.0:
                self.publish_icp_transform(self.latest_correction, msg)
            return
        
        # Transform new scan to the map frame
        new_scan.transform(T_odom_lidar)
        # if self.count == 10:
        new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(0, 255, 0))  # green
        self.new_scan_pub.publish(new_msg)


        # ICP Convergence threshold
        threshold = 0.12
        # Run ICP: Align new_scan (source) to accumulated_map (target)
        icp_result = o3d.pipelines.registration.registration_icp(
            new_scan, valid_reference, threshold, self.latest_correction,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=12000))
        

        #consider using fitness to choose activ/new reference frame
        self.get_logger().info(f'inlier: {icp_result.inlier_rmse}, fitness: {icp_result.fitness}',)
        if icp_result.inlier_rmse > 0.17 or icp_result.fitness < threshold_fitness:
            # self.get_logger().warn("ICP registration did not match well, skipping update.")
            self.icp_publish_counter += 1.0
            if self.icp_publish_counter % 1.0 == 0.0:
                self.publish_icp_transform(self.latest_correction, msg)
            return

        # Update the correction transform
        self.latest_correction = icp_result.transformation
       
        
        self.icp_publish_counter += 1.0
        if self.icp_publish_counter % 1.0 == 0.0:
            self.get_logger().info("uppdate")
            self.publish_icp_transform(self.latest_correction, msg)

            #consider replacing this with the transform from the icp directly after debugging
            if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(seconds=0)):
                tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
                T_map_odom = self.transform_to_matrix(tf_map_odom)

            else:
                # self.get_logger().warn("TF map-odom Lookup failed: skipping update")
                self.icp_publish_counter += 1.0
                if self.icp_publish_counter % 1.0 == 0.0:
                    self.publish_icp_transform(self.latest_correction, msg)
                return
            new_scan.transform(T_map_odom)

            self.count = self.count + 1
            new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(255, 0, 0))  #red
            self.new_scan_icp_pub.publish(new_msg)
            self.merged_pc = None
            self.pc_counter = 0

        # Always republish the first scan
        if self.first_scan is not None:
            first_msg = open3d_to_colored_pointcloud2(self.first_scan, rgb_color=(0, 0, 255))  # Blue
            self.first_scan_pub.publish(first_msg)

    def get_last_reference(self):
        if self.last_ref_from_special and 0 <= self.last_used_ref_index < len(self.special_references):
            return self.special_references[self.last_used_ref_index][0]
        elif not self.last_ref_from_special and 0 <= self.last_used_ref_index < len(self.reference_scans):
            return self.reference_scans[self.last_used_ref_index][0]
        else:
            return None


    def cmd_vel_callback(self, msg):
        self.latest_linear = (msg.linear.x, msg.linear.y)
        self.latest_angular = msg.angular.z

    
    def is_robot_still(self):
        vx, vy = self.latest_linear
        w = self.latest_angular
        return abs(vx) < self.still_threshold_lin and abs(vy) < self.still_threshold_lin and abs(w) < self.still_threshold_ang


    def omega_callback(self, msg):
        self.omega = msg.data

    def find_valid_reference(self, current_pos):
        for i, (ref_pc, ref_pos) in enumerate(self.reference_scans):
            dx = current_pos[0] - ref_pos[0]
            dy = current_pos[1] - ref_pos[1]
            if (dx**2 + dy**2)**0.5 < self.reference_radius:
                return ref_pc, i  # Return the point cloud and index
        return None, -1




    def transform_to_matrix(self, tf_msg):
        """Converts a ROS2 TransformStamped to a 4x4 transformation matrix."""
        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation

        # Convert quaternion to a 3x3 rotation matrix
        r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

        # Construct the 4x4 transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = r
        matrix[:3, 3] = [trans.x, trans.y, trans.z]

        return matrix


    def publish_reference_zones(self, active_index):
        if self.waiting_for_stillness:
            special = self.last_ref_from_special
        else:
            special = self.use_special_refs
        for i, (_, ref_pos) in enumerate(self.reference_scans):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "reference_zones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = ref_pos[0]
            marker.pose.position.y = ref_pos[1]
            marker.pose.position.z = 0.0  # 2D

            marker.pose.orientation.w = 1.0  # No rotation

            marker.scale.x = self.reference_radius * 2.0
            marker.scale.y = self.reference_radius * 2.0
            marker.scale.z = 0.01  # Thin cylinder

            if i == active_index and not special:
                # Active zone — green
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5
            else:
                # Inactive zone — red
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.2

            marker.lifetime.sec = 0  # Keep until overwritten
            self.marker_pub.publish(marker)

        # Special reference zones
        for i, (_, ref_pos) in enumerate(self.special_references):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "special_reference_zones"
            marker.id = 1000 + i  # avoid ID conflict
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = ref_pos[0]
            marker.pose.position.y = ref_pos[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.special_ref_radius * 2.0
            marker.scale.y = self.special_ref_radius * 2.0
            marker.scale.z = 0.01
            if i == active_index and special:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.2
            marker.lifetime.sec = 0
            self.marker_pub.publish(marker)


    def publish_icp_transform(self, transform, msg):
        """Publishes ICP correction directly to /tf."""
        # get latest odom transform

        odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0))
    
        t = TransformStamped()
        t.header.stamp = odom_transform.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = transform[0, 3]
        t.transform.translation.y = transform[1, 3]
        t.transform.translation.z = 0.0  # 2D

        r = R.from_matrix(transform[0:3, 0:3])
        quat = r.as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat

        # **Broadcast the transform to /tf**
        self.tf_broadcaster.sendTransform(t)

        ## self.get_logger().info("Published ICP correction to /tf.")

def main():
    rclpy.init()
    node = IcpMappingNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

