#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class lidar_processing(Node):
    '''
    if the "lidar_launch.yaml" is launched, we will have topic "/scan"
    the node try to listen to /scan topic, read msg, transform it to point cloud 
    and publish(with time stamped and tf frame)
    '''

    def __init__(self):
        super().__init__('lidar_processing')

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_listener_callback,
            10
        )
        self.subscription


    def lidar_listener_callback(self, msg):
        print(f"Received lidar msg, time {msg.header.stamp.sec}")
        '''
        a classic lidar msg has structure below:
        header:
            stamp:
                sec: 1738594286
                nanosec: 567656115
            frame_id: lidar_link
        angle_min: -3.1415927410125732
        angle_max: 3.1415927410125732
        angle_increment: 0.017501909285783768
        time_increment: 0.0003824508748948574
        scan_time: 0.13729986548423767
        range_min: 0.05000000074505806
        range_max: 16.0
        ranges:
        - 0.4000000059604645
        - 0.4009999930858612
        - .inf
        - 0.4000000059604645
        - 0.4000000059604645
        - '...'
        intensities:
        - 47.0
        - 47.0
        - 0.0
        - 47.0
        - 47.0
        - '...'
        '''

def main():
    rclpy.init()
    node = lidar_processing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()