#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import Encoders
import math
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path
from tf2_ros import TransformStamped
from tf2_ros import Buffer, TransformListener

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Initialize the transform broadcaster
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self, spin_thread=False)

        # Initialize the path publisher
        self.path_pub = self.create_publisher(Path, 'path', 10)

        self.create_timer(0.1, self.publish_path)

    def publish_path(self):
        """ Publishes the updated pose. """
        # Get latest transform from odom to base_link
        try:
            trans = self.buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as ex:
            return

        pose = PoseStamped()
        pose.header.stamp = trans.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation.x = trans.transform.rotation.x
        pose.pose.orientation.y = trans.transform.rotation.y
        pose.pose.orientation.z = trans.transform.rotation.z
        pose.pose.orientation.w = trans.transform.rotation.w

        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
