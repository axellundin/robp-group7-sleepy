import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles
import numpy as np


class simple_move(Node):
    """
    this Node will check controller signal,
    process and then publish it to robot wheels.
    """

    def __init__(self):
        super().__init__('simple_move')

        self.subscription = self.create_subscription(
        Twist,
        '/cmd_vel',
        self.joy_stick_calback,
        10)
        self.subscription

        self._pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        #self.bias = 0.004

    def joy_stick_calback(self, msg):
        motor_msg = DutyCycles()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.header.frame_id = "motors"
        duty_cycle_max = 0.5

        motor_msg.duty_cycle_right =  duty_cycle_max * np.max([-1 ,np.min([1, msg.linear.x + msg.angular.z] )]) 
        
        motor_msg.duty_cycle_left = duty_cycle_max *  np.max([-1,np.min([1, msg.linear.x-msg.angular.z] )] )
        # if motor_msg.duty_cycle_left > 0:
        #     motor_msg.duty_cycle_left += self.bias
        # elif motor_msg.duty_cycle_left < 0:
        #     motor_msg.duty_cycle_left -= self.bias
            
        print("Höger")
        print(motor_msg.duty_cycle_right)
        print("Vänster:")
        print(motor_msg.duty_cycle_left)
        self._pub.publish(motor_msg)
        

def main():
    rclpy.init()
    node = simple_move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

        