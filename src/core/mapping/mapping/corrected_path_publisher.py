import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class CorrectedPathPublisher(Node):
    def __init__(self):
        super().__init__('corrected_path_publisher')

        # Subscribe to the uncorrected odometry path
        self.odom_path_sub = self.create_subscription(
            Path, '/path', self.odom_path_callback, 10)

        # Subscribe to ICP transform (map → odom correction)
        self.icp_sub = self.create_subscription(
            TransformStamped, '/icp_transform', self.icp_transform_callback, 10)

        # Publisher for corrected path in the map frame
        self.corrected_path_pub = self.create_publisher(Path, '/corrected_path', 10)

        # TF buffer and listener (for other transforms if needed)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        # Storage for corrected path
        self.corrected_path = Path()
        self.corrected_path.header.frame_id = "map"

        # Store the latest correction transform (initialize as identity)
        self.latest_icp_transform = np.eye(4)

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

        self.latest_icp_transform = matrix  # Store latest correction

    def odom_path_callback(self, msg):
        """Transforms odom-based path to the map frame and publishes corrected path."""
        rclpy.spin_once(self, timeout_sec=0.001)  # Process pending messages

        # If no correction transform has been received, do nothing
        if np.allclose(self.latest_icp_transform, np.eye(4)):  
            self.get_logger().warn("No ICP correction received yet, skipping path correction.")
            return

        # Update header
        self.corrected_path.header.stamp = msg.header.stamp

        # Transform each pose to the map frame using the latest ICP correction
        transformed_poses = []
        for pose in msg.poses:
            p = pose.pose.position
            q = pose.pose.orientation

            # Convert pose to a 4x4 transformation matrix
            r = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            pose_matrix = np.eye(4)
            pose_matrix[:3, :3] = r
            pose_matrix[:3, 3] = [p.x, p.y, p.z]

            # Apply latest ICP correction
            transformed_pose_matrix = np.dot(self.latest_icp_transform, pose_matrix)

            # Convert back to PoseStamped
            new_pose = PoseStamped()
            new_pose.header = msg.header
            new_pose.pose.position.x = transformed_pose_matrix[0, 3]
            new_pose.pose.position.y = transformed_pose_matrix[1, 3]
            new_pose.pose.position.z = 0.0  # 2D assumption

            new_quat = R.from_matrix(transformed_pose_matrix[:3, :3]).as_quat()
            new_pose.pose.orientation.x = new_quat[0]
            new_pose.pose.orientation.y = new_quat[1]
            new_pose.pose.orientation.z = new_quat[2]
            new_pose.pose.orientation.w = new_quat[3]

            transformed_poses.append(new_pose)

        # Update and publish the corrected path
        self.corrected_path.poses = transformed_poses
        self.corrected_path_pub.publish(self.corrected_path)

def main():
    rclpy.init()
    node = CorrectedPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
