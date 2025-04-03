#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from collections import deque
from robp_interfaces.msg import Encoders

class TickAccumulator(Node):
    def __init__(self):
        super().__init__('tick_accumulator')

        self.last_stamp = None 
        self.last_ticks_left = 0 
        self.last_ticks_right = 0 
        self.accumulated_delta_ticks_left = 0 
        self.accumulated_delta_ticks_right = 0 
        self.accumulated_dt = 0 

        self.update_frequency = 30

        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 100)

        # Publisher for accumulated ticks
        self.tick_pub = self.create_publisher(Encoders, '/accumulated_ticks', 10)

    def encoder_callback(self, msg):
        """Accumulate encoder ticks from each message."""
        
        ticks_left = msg.encoder_left 
        ticks_right = msg.encoder_right  

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

        if self.accumulated_dt > 1 / self.update_frequency:
            self.publish_accumulated_ticks()
            self.accumulated_dt = 0
            self.accumulated_delta_ticks_left = 0
            self.accumulated_delta_ticks_right = 0

    def publish_accumulated_ticks(self):
        tick_msg = Encoders()
        tick_msg.header.stamp = self.last_stamp
        tick_msg.encoder_left = self.accumulated_delta_ticks_left
        tick_msg.encoder_right = self.accumulated_delta_ticks_right
        self.tick_pub.publish(tick_msg)

def main():
    rclpy.init()
    node = TickAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
