# #!/usr/bin/env python

# import math

# import numpy as np

# import rclpy
# from rclpy.node import Node

# from tf2_ros import TransformBroadcaster

# from geometry_msgs.msg import TransformStamped
# from robp_interfaces.msg import Encoders
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped

# def quaternion_from_yaw(yaw):
#     """
#     Convert yaw angle to quaternion, for 2D rotation only.
#     """
#     return [0.0,                  # x
#             0.0,                  # y
#             math.sin(yaw / 2.0),  # z
#             math.cos(yaw / 2.0)]  # w

# class Odometry(Node):

#     def __init__(self):
#         super().__init__('odometry')
#         print("original_odometry encoder started")

#         # Initialize the transform broadcaster
#         self._tf_broadcaster = TransformBroadcaster(self)

#         # Initialize the path publisher
#         self._path_pub = self.create_publisher(Path, 'path', 10)
#         # Store the path here
#         self._path = Path()

#         # Subscribe to encoder topic and call callback function on each recieved message
#         self.create_subscription(
#             Encoders, '/motor/encoders', self.encoder_callback, 10)

#         # 2D pose
#         self._x = 0.0
#         self._y = 0.0
#         self._yaw = 0.0

#     def encoder_callback(self, msg: Encoders):
#         """Takes encoder readings and updates the odometry.

#         This function is called every time the encoders are updated (i.e., when a message is published on the '/motor/encoders' topic).

#         Your task is to update the odometry based on the encoder data in 'msg'. You are allowed to add/change things outside this function.

#         Keyword arguments:
#         msg -- An encoders ROS message. To see more information about it 
#         run 'ros2 interface show robp_interfaces/msg/Encoders' in a terminal.
#         """

#         # The kinematic parameters for the differential configuration
#         dt = 50 / 1000
#         ticks_per_rev = 48 * 64
#         wheel_radius = 0.04921  # TODO: Fill in
#         base = 0.3  # TODO: Fill in

#         # Ticks since last message
#         delta_ticks_left = msg.delta_encoder_left
#         delta_ticks_right = msg.delta_encoder_right

#         dist_l = 2 * math.pi * wheel_radius * delta_ticks_left / ticks_per_rev
#         dist_r = 2 * math.pi * wheel_radius * delta_ticks_right / ticks_per_rev

#         vel_l = dist_l / dt
#         vel_r = dist_r / dt

#         vel = (vel_l+vel_r)/2
#         omega = (vel_r - vel_l) / base

#         self._yaw = self._yaw + omega * dt
#         self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
#         self.average_yaw = self._yaw - omega*dt/2

#         self._x = self._x + vel * dt * math.cos(self.average_yaw) 
#         self._y = self._y + vel * dt * math.sin(self.average_yaw) 
        
#         stamp = msg.header.stamp 

#         self.broadcast_transform(stamp, self._x, self._y, self._yaw)
#         self.publish_path(stamp, self._x, self._y, self._yaw)


#     def broadcast_transform(self, stamp, x, y, yaw):
#         """Takes a 2D pose and broadcasts it as a ROS transform.

#         Broadcasts a 3D transform with z, roll, and pitch all zero. 
#         The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.

#         Keyword arguments:
#         stamp -- timestamp of the transform
#         x -- x coordinate of the 2D pose
#         y -- y coordinate of the 2D pose
#         yaw -- yaw of the 2D pose (in radians)
#         """

#         t = TransformStamped()
#         t.header.stamp = stamp # stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'

#         # The robot only exists in 2D, thus we set x and y translation
#         # coordinates and set the z coordinate to 0
#         t.transform.translation.x = x
#         t.transform.translation.y = y
#         t.transform.translation.z = 0.0

#         # For the same reason, the robot can only rotate around one axis
#         # and this why we set rotation in x and y to 0 and obtain
#         # rotation in z axis from the message
#         q = quaternion_from_yaw(yaw)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]

#         # Send the transformation
#         self._tf_broadcaster.sendTransform(t)

#     def publish_path(self, stamp, x, y, yaw):
#         """Takes a 2D pose appends it to the path and publishes the whole path.

#         Keyword arguments:
#         stamp -- timestamp of the transform
#         x -- x coordinate of the 2D pose
#         y -- y coordinate of the 2D pose
#         yaw -- yaw of the 2D pose (in radians)
#         """

#         self._path.header.stamp = stamp
#         self._path.header.frame_id = 'odom'

#         pose = PoseStamped()
#         pose.header = self._path.header

#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

#         q = quaternion_from_yaw(yaw)
#         pose.pose.orientation.x = q[0]
#         pose.pose.orientation.y = q[1]
#         pose.pose.orientation.z = q[2]
#         pose.pose.orientation.w = q[3]

#         self._path.poses.append(pose)

#         print(f"Publishing: x={x}, y={y}, yaw={yaw}")
#         self._path_pub.publish(self._path)

# def main():
#     rclpy.init()
#     node = Odometry()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()











# # !/usr/bin/env python

# import math
# import numpy as np
# from math import atan2
# import rclpy
# from rclpy.node import Node

# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped, Quaternion
# from robp_interfaces.msg import Encoders
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Imu


# def quaternion_from_euler(roll, pitch, yaw):
#     """Convert Euler angles to quaternion."""
#     qx = 0.0
#     qy = 0.0
#     qz = math.sin(yaw / 2)
#     qw = math.cos(yaw / 2)
#     return [qx, qy, qz, qw]


# class Odometry(Node):

#     def __init__(self):
#         super().__init__('odometry')
#         print("latest odom encoder +imu")

#         # Initialize the transform broadcaster
#         self._tf_broadcaster = TransformBroadcaster(self)

#         # Initialize the path publisher
#         self._path_pub = self.create_publisher(Path, 'path', 10)
#         self._pose_pub = self.create_publisher(PoseStamped, 'robot_current_pose', 10)

#         # Store the path here
#         self._path = Path()

#         # Subscribe to encoder and IMU topic
#         self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)
#         self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)

#         # Initialize pose variables
#         self._x = 0.0
#         self._y = 0.0
#         self._yaw = 0.0

#         # IMU handling
#         self.q = Quaternion()
#         self.init = True
#         self.zero_value = 0

#     def encoder_callback(self, msg: Encoders):
#         """Process encoder readings to update odometry."""

#         # Define kinematic parameters
#         dt = 50 / 1000  # 50 ms
#         ticks_per_rev = 48 * 64
#         wheel_radius = 0.04921
#         base = 0.3  # Wheelbase distance

#         # Compute wheel displacement
#         delta_ticks_left = msg.delta_encoder_left
#         delta_ticks_right = msg.delta_encoder_right
#         k = (2 * math.pi) / ticks_per_rev
#         D = (wheel_radius / 2) * k * (delta_ticks_right + delta_ticks_left)
#         angular_from_w = wheel_radius * k * 20 * (delta_ticks_right - delta_ticks_left) / base
#         d_theta = angular_from_w * dt # We can use this if we decide to fuse in a nontrivial way, for //Yassir
       

#         # Update pose
#         self._x += D * math.cos(self._yaw)
#         self._y += D * math.sin(self._yaw)

#         if self.init:
#             self.zero_value = -self.get_yaw(self.q) - math.pi / 2
#             self.init = False
#         else:
#             self._yaw = -self.get_yaw(self.q) - self.zero_value  # Update yaw from IMU, can be improved

#         stamp = msg.header.stamp

#         self.broadcast_transform(stamp, self._x, self._y, self._yaw)
#         self.publish_path(stamp, self._x, self._y, self._yaw)

#     def imu_callback(self, msg):
#         """Update quaternion from IMU."""
#         self.q = msg.orientation

#     def broadcast_transform(self, stamp, x, y, yaw):
#         """Broadcast odometry transform."""
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'

#         t.transform.translation.x = x
#         t.transform.translation.y = y
#         t.transform.translation.z = 0.0

#         q = quaternion_from_euler(0.0, 0.0, yaw)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]

#         self._tf_broadcaster.sendTransform(t)

#     def publish_path(self, stamp, x, y, yaw):
#         """Publish robot path."""
#         self._path.header.stamp = stamp
#         self._path.header.frame_id = 'odom'

#         pose = PoseStamped()
#         pose.header = self._path.header
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = 0.01  # 1 cm above ground

#         q = quaternion_from_euler(0.0, 0.0, yaw)
#         pose.pose.orientation.x = q[0]
#         pose.pose.orientation.y = q[1]
#         pose.pose.orientation.z = q[2]
#         pose.pose.orientation.w = q[3]

#         self._path.poses.append(pose)

#         self._path_pub.publish(self._path)
#         self._pose_pub.publish(pose)

#     def get_yaw(self, q):
#         """Extract yaw from quaternion."""
#         return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# def main():
#     rclpy.init()
#     node = Odometry()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# """x: -0.004630488343536854
#   y: 0.010369664989411831
#   z: -0.938453197479248
#   w: -0.34521979093551636
  
#   x: -0.014878804795444012
#   y: 0.003655950538814068
#   z: -0.8942620754241943
#   w: 0.4472814202308655
  
#   orientation:
#   x: -0.0021696151234209538
#   y: 0.007577390875667334
#   z: -0.9879505038261414
#   w: 0.15456923842430115
#   """




















#!/usr/bin/env python

import math
import numpy as np
from math import atan2
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        self._pose_pub = self.create_publisher(PoseStamped, 'robot_current_pose', 10)

        # Store the path here
        self._path = Path()

        # Subscribe to encoder and IMU topic
        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)

        # Initialize pose variables
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # IMU handling
        self.q = Quaternion()
        self.init = True
        self.zero_value = 0

    def encoder_callback(self, msg: Encoders):
        """Process encoder readings to update odometry."""

        # Define kinematic parameters
        dt = 50 / 1000  # 50 ms
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04921
        base = 0.31  # Wheelbase distance

        # Compute wheel displacement
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right
        k = (2 * math.pi) / ticks_per_rev
        D = (wheel_radius / 2) * k * (delta_ticks_right + delta_ticks_left)
        angular_w = wheel_radius * k * 20 * (delta_ticks_right - delta_ticks_left) / base
        d_theta = angular_w * dt

        # Update pose
        self._x += D * math.cos(self._yaw)
        self._y += D * math.sin(self._yaw)

        if self.init:
            self.zero_value = -self.get_yaw(self.q) - math.pi / 2
            self.init = False
        else:
            self._yaw = -self.get_yaw(self.q) - self.zero_value  # Update yaw from IMU

        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_path(stamp, self._x, self._y, self._yaw)

    def imu_callback(self, msg):
        """Update quaternion from IMU."""
        self.q = msg.orientation

    def broadcast_transform(self, stamp, x, y, yaw):
        """Broadcast odometry transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        """Publish robot path."""
        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm above ground

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        self._path_pub.publish(self._path)
        self._pose_pub.publish(pose)

    def get_yaw(self, q):
        """Extract yaw from quaternion."""
        return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
