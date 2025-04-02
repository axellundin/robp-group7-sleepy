#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import Encoders
import math
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path
from tf2_ros import TransformStamped

class OdometryTest(Node):
    def __init__(self):
        super().__init__('odometry_test')

        self.accumulated_ticks_left = 0
        self.accumulated_ticks_right = 0
        group = ReentrantCallbackGroup()

        self.create_subscription(
            Encoders, 
            '/motor/encoders', 
            self.encoder_callback, 
            100, 
            callback_group=group) 
        
         # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self.path_pub = self.create_publisher(Path, 
                                               'path', 
                                               10)
        
        self.Q = []
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.accumulated_delta_ticks_left = 0 
        self.accumulated_delta_ticks_right = 0 
        self.accumulated_dt = 0 

        self.last_ticks_left = None 
        self.last_ticks_right = None
        self.last_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        group2 =   MutuallyExclusiveCallbackGroup()
        self.executor = None
        self.alive = True
        self.create_timer(0.1, self.update_pose, callback_group=group2)
        # start_new_thread(self.manual_timer, ())

    # def manual_timer(self): 
    #     while self.alive: 
    #         self.update_pose()
    #         self.executor.spin_once(timeout_sec=0.05)

    def encoder_callback(self, msg):
        """Accumulate encoder ticks from each message."""

        ticks_left = msg.encoder_left 
        ticks_right = msg.encoder_right  

        # Only runs once, does not add a delta to the queue 
        if self.last_stamp is None: 
            self.last_stamp = msg.header.stamp 
            self.last_ticks_left = ticks_left  
            self.last_ticks_right = ticks_right 
            return 
        
        t_new = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_old = self.last_stamp.sec + self.last_stamp.nanosec * 1e-9
        dt = t_new - t_old
        if dt == 0: 
            return 

        delta_ticks_left = ticks_left - self.last_ticks_left 
        delta_ticks_right = ticks_right - self.last_ticks_right 

        self.last_stamp = msg.header.stamp 
        self.last_ticks_left = ticks_left  
        self.last_ticks_right = ticks_right 

        self.accumulated_delta_ticks_left += delta_ticks_left 
        self.accumulated_delta_ticks_right += delta_ticks_right
        self.accumulated_dt += dt 

        # self.Q.append([delta_ticks_left, delta_ticks_right, dt, msg.header.stamp])

    def update_pose(self): 
        """ Integrates all values in the queue and publishes the updated pose. """ 
        time = None 
        # Q_copy = self.Q.copy()
        # self.Q = []
        # # print(f"Length of Q: {len(Q_copy)}")
        # if len(Q_copy) == 0: 
        #     return
        
        # accumulated_delta_ticks_left = 0 
        # accumulated_delta_ticks_right = 0 
        # accumulated_dt = 0 

        # for T in Q_copy: 
        #     ticks_left, ticks_right, dt, time = T
        #     accumulated_delta_ticks_left += ticks_left
        #     accumulated_delta_ticks_right += ticks_right
        #     accumulated_dt += dt 

        accumulated_delta_ticks_left = self.accumulated_delta_ticks_left
        accumulated_delta_ticks_right = self.accumulated_delta_ticks_right 
        accumulated_dt = self.accumulated_dt  
        time = self.last_stamp
        
        if accumulated_dt == 0: 
            return 
        
        self.accumulated_delta_ticks_left = 0  
        self.accumulated_delta_ticks_right = 0 
        self.accumulated_dt = 0 
        
        delta_x, delta_y, delta_theta = self.compute_pose_delta(accumulated_delta_ticks_left, accumulated_delta_ticks_right, accumulated_dt)
        mid_theta = self.theta + delta_theta / 2

        self.x += delta_x * math.cos(mid_theta)
        self.y += delta_y * math.sin(mid_theta)
        self.theta += delta_theta 
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # print(f'Publishing pose: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}')
        time = self.get_clock().now().to_msg()
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
        return delta_x, delta_y, delta_theta 

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]

def main():
    rclpy.init()
    node = OdometryTest()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    node.executor = ex
    node.alive = True
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.alive = False
    rclpy.shutdown()


if __name__ == '__main__':
    main()
