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

    def joy_stick_calback(self, msg):
        motor_msg = DutyCycles()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.header.frame_id = "motors"
        B = 0.3124
        Vmax = 300
        wmax = 100
        r = 0.04921
        K = 3072 * 2*np.pi

        motor_msg.duty_cycle_right = (1/(K*r))*(Vmax*msg.linear.x+wmax*msg.angular.z)
        motor_msg.duty_cycle_left = (1/(K*r))*(Vmax*msg.linear.x-wmax*msg.angular.z)
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

        