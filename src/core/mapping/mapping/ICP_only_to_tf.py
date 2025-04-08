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
    radius = 0.2  # 20 cm neighborhood radius
    kdtree = cKDTree(filtered_points)
    to_keep = np.ones(filtered_points.shape[0], dtype=bool)

    for i, p in enumerate(filtered_points):
        if not to_keep[i]:
            continue  # Already marked for removal
        idxs = kdtree.query_ball_point(p, r=radius)
        idxs.remove(i)  # Don't remove itself
        to_keep[idxs] = False  # Remove all close neighbors

    final_points = filtered_points[to_keep]
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
            LaserScan, '/scan', self.scan_callback, 10, callback_group=group)
        
        self.ref_scan_pub = self.create_publisher(PointCloud2, '/reference_scan', 10)
        self.new_scan_pub = self.create_publisher(PointCloud2, '/new_scan', 10)
        self.new_scan_icp_pub = self.create_publisher(PointCloud2, '/new_scan_icp', 10)


        # # Subscribe to ICP transform (use latest correction instead of tf lookup)
        # self.icp_sub = self.create_subscription(
        #     TransformStamped, '/tf', self.icp_transform_callback, 10)

        # **TF Broadcaster instead of topic publisher**
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        # Store the latest correction transform (initialize as identity)
        self.latest_correction = None

        self.accumulated_map = None  # Store the global map in `map` frame

    # def icp_transform_callback(self, msg):
    #     """Updates the latest correction transform from /tf."""
    #     self.get_logger().info("Received new ICP correction.")

    #     # Convert TransformStamped to 4x4 matrix
    #     trans = msg.transform.translation
    #     rot = msg.transform.rotation

    #     r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
    #     matrix = np.eye(4)
    #     matrix[:3, :3] = r
    #     matrix[:3, 3] = [trans.x, trans.y, trans.z]

    #     self.latest_correction = matrix  # Store latest correction
    #     self.has_correction = True  # Mark that we have a correction

        # reference scan
        self.reference_scan = None
        self.count = 0
        self.icp_publish_counter = 0


    def scan_callback(self, msg):
        """Transforms new scan to map frame, performs ICP, and updates transform."""

        if self.latest_correction is None:
            self.latest_correction = np.eye(4)
            self.publish_icp_transform(self.latest_correction, msg)
            return


        new_scan = convert_scan_to_open3d(msg)

        # rclpy.spin_once(self, timeout_sec=0.001)  # Process TF messages

        # Get transform from lidar → odom (static transform)
        # try:
        #     tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
        # except tf2_ros.LookupException:
        #     self.get_logger().warn("TF Lookup failed: Could not get odom → lidar_link transform.")
        #     self.publish_icp_transform(self.latest_correction if self.has_correction else np.eye(4), msg)
        #     return
        
        # # rclpy.spin_once(self, timeout_sec=0.001)

        # try:
        #     tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
        #     # Convert TransformStamped to 4x4 matrix
        #     T_map_odom = self.transform_to_matrix(tf_map_odom)
        #     self.has_correction = True  # Mark that we have a correction
        # except tf2_ros.LookupException:
        #     self.get_logger().warn("TF Lookup failed: Using identity transform for map → odom.")
        #     T_map_odom = np.eye(4)  # Identity transform if lookup fails
        

        # # Convert TransformStamped to 4x4 matrix
        # T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)

        # # Compute final transform: T_map_scan = T_map_odom * T_odom_lidar
        # T_map_scan = np.dot(T_map_odom, T_odom_lidar)

        # # Single lookup: Get transform from lidar → map
        # try:
        #     tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
        #     T_map_lidar = self.transform_to_matrix(tf_map_lidar)
        #     self.has_correction = True  # Mark that we have a correction
        # except tf2_ros.LookupException and tf2_ros.ConnectivityException:
        #     self.get_logger().warn("TF Lookup failed: Using identity transform for map → lidar_link.")
        #     T_map_lidar = np.eye(4)  # Use identity transform if lookup fails

        if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
            tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
            # print(tf_map_lidar)
            T_map_lidar = self.transform_to_matrix(tf_map_lidar)
        else:
            self.get_logger().warn("TF Lookup failed: skipping update")
            self.icp_publish_counter += 1
            if self.icp_publish_counter % 1 == 0:
                self.publish_icp_transform(self.latest_correction, msg)
            return
            
        # Transform new scan to the map frame
        new_scan.transform(T_map_lidar)

        if self.reference_scan is None:
            # Store the first scan in the map frame
            self.reference_scan = new_scan
            # publishes ref scan in map frame
            ref_msg = open3d_to_pointcloud2(self.reference_scan)
            self.ref_scan_pub.publish(ref_msg)
            # Keeps publishing constant
            self.publish_icp_transform(self.latest_correction, msg)
            return
        # if self.count == 5:
        # new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(0, 255, 0))  # green
        # self.new_scan_pub.publish(new_msg)


        # ICP Convergence threshold
        threshold = 0.1
        # Run ICP: Align `new_scan` (source) to `accumulated_map` (target)
        icp_result = o3d.pipelines.registration.registration_icp(
            self.reference_scan, new_scan, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        
        # Check if ICP was successful
        # print(icp_result.fitness)
        # print(icp_result.inlier_rmse)

        # if icp_result.fitness < 0.85:
        # self.get_logger().info(f"ICP inlier RMSE: {icp_result.inlier_rmse}")
        # print(f"ICP inlier RMSE: {icp_result.inlier_rmse}")
        self.get_logger().info(f'inlier: {icp_result.inlier_rmse}, fitness: {icp_result.fitness}',)
        if icp_result.inlier_rmse > 0.01 or icp_result.fitness < 0.6:
            #self.get_logger().warn("ICP registration did not match well, skipping update.")
            self.icp_publish_counter += 1
            if self.icp_publish_counter % 1 == 0:
                self.publish_icp_transform(self.latest_correction, msg)
            return

        # Update the correction transform
        #self.latest_correction = np.dot(icp_result.transformation, self.latest_correction)
        self.latest_correction = icp_result.transformation
        # # Apply the ICP transformation to align the new scan
        # new_scan.transform(icp_result.transformation)

        # # Merge new scan into accumulated map
        # self.accumulated_map += new_scan

        # # Perform voxel downsampling to reduce the number of points
        # self.accumulated_map = self.accumulated_map.voxel_down_sample(voxel_size=0.05)

        # Publish the correction transform to `/tf` instead of `/icp_transform`
        self.icp_publish_counter += 1
        if self.icp_publish_counter % 1 == 0:
            self.get_logger().info("uppdate")
            self.publish_icp_transform(self.latest_correction, msg)
            self.count = self.count + 1
            if self.count == 5:
                new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(255, 0, 0))  #red
                self.new_scan_icp_pub.publish(new_msg)

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

        #self.get_logger().info("Published ICP correction to /tf.")

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

#     # --- Region filter parameters ---
#     a = 0.085
#     b = 0.12
#     r = 0.39
#     q = 0.31

#     # Apply region mask
#     x_scaled = (x + a) / r
#     y_scaled = (y + b) / q
#     mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) >= 1


#     filtered_points = np.column_stack((x[mask], y[mask], np.zeros_like(x[mask])))

#     # Convert to Open3D point cloud
#     pc = o3d.geometry.PointCloud()
#     pc.points = o3d.utility.Vector3dVector(filtered_points)
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
#             LaserScan, '/scan', self.scan_callback, 10, callback_group=group)
        
#         self.ref_scan_pub = self.create_publisher(PointCloud2, '/reference_scan', 10)
#         self.new_scan_pub = self.create_publisher(PointCloud2, '/new_scan', 10)
#         self.new_scan_icp_pub = self.create_publisher(PointCloud2, '/new_scan_icp', 10)


#         # # Subscribe to ICP transform (use latest correction instead of tf lookup)
#         # self.icp_sub = self.create_subscription(
#         #     TransformStamped, '/tf', self.icp_transform_callback, 10)

#         # **TF Broadcaster instead of topic publisher**
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # TF buffer and listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

#         # Store the latest correction transform (initialize as identity)
#         self.latest_correction = None

#         self.accumulated_map = None  # Store the global map in `map` frame

#     # def icp_transform_callback(self, msg):
#     #     """Updates the latest correction transform from /tf."""
#     #     self.get_logger().info("Received new ICP correction.")

#     #     # Convert TransformStamped to 4x4 matrix
#     #     trans = msg.transform.translation
#     #     rot = msg.transform.rotation

#     #     r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
#     #     matrix = np.eye(4)
#     #     matrix[:3, :3] = r
#     #     matrix[:3, 3] = [trans.x, trans.y, trans.z]

#     #     self.latest_correction = matrix  # Store latest correction
#     #     self.has_correction = True  # Mark that we have a correction

#         # reference scan
#         self.reference_scan = None
#         self.count = 0
#         self.icp_publish_counter = 0


#     def scan_callback(self, msg):
#         """Transforms new scan to map frame, performs ICP, and updates transform."""

#         if self.latest_correction is None:
#             self.latest_correction = np.eye(4)
#             self.publish_icp_transform(self.latest_correction, msg)
#             return


#         new_scan = convert_scan_to_open3d(msg)

#         # rclpy.spin_once(self, timeout_sec=0.001)  # Process TF messages

#         # Get transform from lidar → odom (static transform)
#         # try:
#         #     tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
#         # except tf2_ros.LookupException:
#         #     self.get_logger().warn("TF Lookup failed: Could not get odom → lidar_link transform.")
#         #     self.publish_icp_transform(self.latest_correction if self.has_correction else np.eye(4), msg)
#         #     return
        
#         # # rclpy.spin_once(self, timeout_sec=0.001)

#         # try:
#         #     tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
#         #     # Convert TransformStamped to 4x4 matrix
#         #     T_map_odom = self.transform_to_matrix(tf_map_odom)
#         #     self.has_correction = True  # Mark that we have a correction
#         # except tf2_ros.LookupException:
#         #     self.get_logger().warn("TF Lookup failed: Using identity transform for map → odom.")
#         #     T_map_odom = np.eye(4)  # Identity transform if lookup fails
        

#         # # Convert TransformStamped to 4x4 matrix
#         # T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)

#         # # Compute final transform: T_map_scan = T_map_odom * T_odom_lidar
#         # T_map_scan = np.dot(T_map_odom, T_odom_lidar)

#         # # Single lookup: Get transform from lidar → map
#         # try:
#         #     tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
#         #     T_map_lidar = self.transform_to_matrix(tf_map_lidar)
#         #     self.has_correction = True  # Mark that we have a correction
#         # except tf2_ros.LookupException and tf2_ros.ConnectivityException:
#         #     self.get_logger().warn("TF Lookup failed: Using identity transform for map → lidar_link.")
#         #     T_map_lidar = np.eye(4)  # Use identity transform if lookup fails

#         if self.tf_buffer.can_transform("map", "lidar_link", rclpy.time.Time(seconds=0)):
#             tf_map_lidar = self.tf_buffer.lookup_transform("map", "lidar_link", rclpy.time.Time(seconds=0))
#             # print(tf_map_lidar)
#             T_map_lidar = self.transform_to_matrix(tf_map_lidar)
#         else:
#             self.get_logger().warn("TF Lookup failed: skipping update")
#             self.icp_publish_counter += 1
#             if self.icp_publish_counter % 2 == 0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return
            
#         # Transform new scan to the map frame
#         new_scan.transform(T_map_lidar)

#         if self.reference_scan is None:
#             # Store the first scan in the map frame
#             self.reference_scan = new_scan
#             #publishes ref scan in map frame
#             ref_msg = open3d_to_pointcloud2(self.reference_scan)
#             self.ref_scan_pub.publish(ref_msg)
#             #Keeps publishing constant
#             self.publish_icp_transform(self.latest_correction, msg)
#             return
#         if self.count == 5:
#             new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(0, 255, 0))  # green
#             self.new_scan_pub.publish(new_msg)


#         # ICP Convergence threshold
#         threshold = 0.05
#         # Run ICP: Align `new_scan` (source) to `accumulated_map` (target)
#         icp_result = o3d.pipelines.registration.registration_icp(
#             self.reference_scan, new_scan, threshold, np.eye(4),
#             o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        
#         # Check if ICP was successful
#         # print(icp_result.fitness)
#         # print(icp_result.inlier_rmse)

#         if icp_result.fitness < 0.85:
#             #self.get_logger().warn("ICP registration did not match well, skipping update.")
#             self.icp_publish_counter += 1
#             if self.icp_publish_counter % 2 == 0:
#                 self.publish_icp_transform(self.latest_correction, msg)
#             return
#         self.count = self.count + 1

        
#         if self.count == 5:
#             new_msg = open3d_to_colored_pointcloud2(new_scan, rgb_color=(255, 0, 0))  # green
#             self.new_scan_icp_pub.publish(new_msg)
#         self.get_logger().info("uppdate")
#         # Update the correction transform
#         self.latest_correction = np.dot(self.latest_correction, icp_result.transformation)

#         # # Apply the ICP transformation to align the new scan
#         # new_scan.transform(icp_result.transformation)

#         # # Merge new scan into accumulated map
#         # self.accumulated_map += new_scan

#         # # Perform voxel downsampling to reduce the number of points
#         # self.accumulated_map = self.accumulated_map.voxel_down_sample(voxel_size=0.05)

        

#         # Publish the correction transform to `/tf` instead of `/icp_transform`
#         self.icp_publish_counter += 1
#         if self.icp_publish_counter % 2 == 0:
#             self.publish_icp_transform(self.latest_correction, msg)

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
#         t.transform.translation.z = 0.0  # Assume 2D

#         r = R.from_matrix(transform[0:3, 0:3])
#         quat = r.as_quat()
#         t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat

#         # **Broadcast the transform to /tf**
#         self.tf_broadcaster.sendTransform(t)

#         #self.get_logger().info("Published ICP correction to /tf.")

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
