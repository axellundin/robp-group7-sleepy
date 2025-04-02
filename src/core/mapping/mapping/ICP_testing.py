import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan

# Convert LiDAR scan to Open3D point cloud
def convert_scan_to_open3d(scan_msg):
    """Converts a ROS2 LaserScan message into an Open3D point cloud."""
    points = []
    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if np.isfinite(r):  # Ensure valid range measurements
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append([x, y, 0.0])  # 2D scan, so z=0
        angle += scan_msg.angle_increment
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(np.array(points))
    return pc

class IcpMappingNode(Node):
    def __init__(self):
        super().__init__('icp_mapping_node')

        # Subscribe to the LiDAR scan topic
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Subscribe to ICP transform (use latest correction instead of tf lookup)
        self.icp_sub = self.create_subscription(
            TransformStamped, '/icp_transform', self.icp_transform_callback, 10)

        # Publisher for ICP corrections
        self.icp_pub = self.create_publisher(TransformStamped, '/icp_transform', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        # Store the latest correction transform (initialize as identity)
        self.latest_correction = np.eye(4)
        self.has_correction = False  # Track if a correction has been received

        self.accumulated_map = None  # Store the global map in `map` frame

    def icp_transform_callback(self, msg):
        """Updates the latest correction transform from /icp_transform."""
        self.get_logger().info("Received new ICP correction.")

        # Convert TransformStamped to 4x4 matrix
        trans = msg.transform.translation
        rot = msg.transform.rotation

        r = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
        matrix = np.eye(4)
        matrix[:3, :3] = r
        matrix[:3, 3] = [trans.x, trans.y, trans.z]

        self.latest_correction = matrix  # Store latest correction
        self.has_correction = True  # Mark that we have a correction

    def scan_callback(self, msg):
        """Transforms new scan to map frame, performs ICP, and updates transform."""
        new_scan = convert_scan_to_open3d(msg)
        rclpy.spin_once(self, timeout_sec=0.001)  # Process TF messages

        # Get transform from lidar → odom (static transform)
        try:
            tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time())
        except tf2_ros.LookupException:
            self.get_logger().warn("TF Lookup failed: Could not get odom → lidar_link transform.")
            return

        # Use identity for first correction; otherwise, use the latest ICP correction
        if not self.has_correction:
            T_map_odom = np.eye(4)  # Identity transform
        else:
            T_map_odom = self.latest_correction  # Latest ICP correction

        # Convert TransformStamped to 4x4 matrix
        T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)

        # Compute final transform: T_map_scan = T_map_odom * T_odom_lidar
        T_map_scan = np.dot(T_map_odom, T_odom_lidar)

        # Transform new scan to the map frame
        new_scan.transform(T_map_scan)

        if self.accumulated_map is None:
            # Store the first scan in the map frame
            self.accumulated_map = new_scan
            return

        # ICP Convergence threshold
        threshold = 0.05

        # Run ICP: Align `new_scan` (source) to `accumulated_map` (target)
        icp_result = o3d.pipelines.registration.registration_icp(
            new_scan, self.accumulated_map, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

        # Check if ICP was successful
        if icp_result.fitness < 0.5:
            self.get_logger().warn("ICP registration did not match well, skipping update.")
            return

        # Update the correction transform
        self.latest_correction = np.dot(self.latest_correction, icp_result.transformation)
        self.has_correction = True  # Mark that we have a correction

        # Apply the ICP transformation to align the new scan
        new_scan.transform(icp_result.transformation)

        # Merge new scan into accumulated map
        self.accumulated_map += new_scan

        # Perform voxel downsampling to reduce the number of points
        self.accumulated_map = self.accumulated_map.voxel_down_sample(voxel_size=0.05)

        # Publish the correction as a ROS topic instead of TF
        self.publish_icp_transform(self.latest_correction, msg)

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
        """Publishes ICP correction on /icp_transform instead of TF."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = transform[0, 3]
        t.transform.translation.y = transform[1, 3]
        t.transform.translation.z = 0.0  # Assume 2D

        r = R.from_matrix(transform[0:3, 0:3])
        quat = r.as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat

        self.icp_pub.publish(t)

def main():
    rclpy.init()
    node = IcpMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
