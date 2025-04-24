#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import Encoders
import math
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path
from tf2_ros import TransformStamped
from std_msgs.msg import Float32


class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self.create_subscription(Encoders, '/accumulated_ticks', self.ticks_callback, 100)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_stamp = None

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.omega_pub = self.create_publisher(Float32, '/angular_velocity', 10)


    def ticks_callback(self, msg):
        """Compute odometry from accumulated ticks."""
        if self.last_stamp is None:
            self.last_stamp = msg.header.stamp
            return
        
        delta_ticks_left = msg.encoder_left 
        delta_ticks_right = msg.encoder_right
        delta_t = (msg.header.stamp.sec - self.last_stamp.sec) + (msg.header.stamp.nanosec - self.last_stamp.nanosec) * 1e-9
        
        # Store current timestamp for next iteration
        self.last_stamp = msg.header.stamp

        delta_x, delta_y, delta_theta, omega = self.compute_pose_delta(delta_ticks_left, delta_ticks_right, delta_t)
        self.omega_pub.publish(Float32(data=omega))
        mid_theta = self.theta + delta_theta / 2

        self.x += delta_x * math.cos(mid_theta)
        self.y += delta_y * math.sin(mid_theta)
        self.theta += delta_theta 
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # time = self.get_clock().now().to_msg()
        time = msg.header.stamp
        # self.publish_path(time, self.x, self.y, self.theta)
        self.broadcast_transform(time, self.x, self.y, self.theta)

    def publish_path(self, time, x, y, theta):
        """ Publishes the updated pose. """
        pose = PoseStamped()
        pose.header.stamp = time

        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def broadcast_transform(self, stamp, x, y, theta):
        """Broadcast odometry transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def compute_pose_delta(self, delta_ticks_left, delta_ticks_right, dt):
        """ Computes the change in pose from the encoder ticks and time step """
        # The kinematic parameters for the differential configuration
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04921  # TODO: Calibrate
        base = 0.309  # TODO: Calibrate

        dist_l = 2 * math.pi * wheel_radius * delta_ticks_left / ticks_per_rev
        dist_r = 2 * math.pi * wheel_radius * delta_ticks_right / ticks_per_rev

        vel_l = dist_l / dt
        vel_r = dist_r / dt

        vel = (vel_l + vel_r) / 2
        omega = (vel_r - vel_l) / base

        delta_theta = omega * dt 
        delta_x = vel * dt 
        delta_y = vel * dt 
        return delta_x, delta_y, delta_theta, omega 

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]

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
