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

        # Publisher for ICP corrections (instead of /tf)
        self.icp_pub = self.create_publisher(TransformStamped, '/icp_transform', 10)

        # TF buffer and listener with spin_thread=False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        self.accumulated_map = None  # Store the global map in `map` frame
        self.current_correction = np.eye(4)  # Transformation correction

    def scan_callback(self, msg):
        """Transforms new scan to map frame, performs ICP, and updates transform."""
        new_scan = convert_scan_to_open3d(msg)

        # Process any pending TF messages before lookup
        rclpy.spin_once(self, timeout_sec=0.001)

        # Get transform from lidar → odom (static transform)
        try:
            tf_odom_lidar = self.tf_buffer.lookup_transform("odom", "lidar_link", rclpy.time.Time(seconds=0))
        except tf2_ros.LookupException:
            self.get_logger().warn("TF Lookup failed: Could not get odom → lidar_link transform.")
            return

        # Process any pending TF messages before the second lookup
        rclpy.spin_once(self, timeout_sec=0.001)

        # Get transform from odom → map (dynamic transform)
        try:
            tf_map_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(seconds=0))
        except tf2_ros.LookupException:
            self.get_logger().warn("TF Lookup failed: Could not get map → odom transform.")
            return

        # Convert TransformStamped to 4x4 transformation matrices
        T_odom_lidar = self.transform_to_matrix(tf_odom_lidar)
        T_map_odom = self.transform_to_matrix(tf_map_odom)

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
        self.current_correction = np.dot(self.current_correction, icp_result.transformation)

        # Apply the ICP transformation to align the new scan
        new_scan.transform(icp_result.transformation)

        # Merge new scan into accumulated map
        self.accumulated_map += new_scan

        # Perform voxel downsampling to reduce the number of points
        self.accumulated_map = self.accumulated_map.voxel_down_sample(voxel_size=0.05)

        # Publish the correction as a ROS topic instead of TF
        self.publish_icp_transform(self.current_correction, msg)


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
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.icp_pub.publish(t)  # Publish to /icp_transform
        self.get_logger().info("Published ICP correction to /icp_transform.")

def main(args=None):
    rclpy.init(args=args)
    node = IcpMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()