import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy

class MoveArmHardcoded(Node):
    def __init__(self):
        super().__init__('move_arm_hardcoded')
        self.get_logger().info('Move arm hardcoded')

        # Create publisher on the topic /multi_servo_cmd_sub
        self.publisher = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.logger = self.get_logger()
        # Create a listener on the topic /joy
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.T = 2000 

        self.up_down_state = 1 # 1 = up -1 = down
        self.gripper_state = 1 # 1 = open -1 = close

        self.gripper_open = 4000
        self.gripper_close = 11000

    def joy_callback(self, msg):
        if msg.buttons[0]: 
            # evaluate state and if there has been a state change, publish the new state
            if self.up_down_state == 1: 
                self.publish_move_arm_down()
                self.up_down_state = -1
        if msg.buttons[3]: 
            if self.up_down_state == -1: 
                self.publish_move_arm_up()
                self.up_down_state = 1
        if msg.buttons[2]: 
            if self.gripper_state == 1: 
                self.publish_close_gripper()
                self.gripper_state = -1
        if msg.buttons[1]: 
            if self.gripper_state == -1: 
                self.publish_open_gripper()
                self.gripper_state = 1

    # Publish the move arm down message 
    def publish_move_arm_down(self): 
        msg = Int16MultiArray()
        msg.data = [
            -1,
            12000, 
            3000, 
            14000, 
            5000, 
            12000,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T]
        self.desired_angles = msg.data[:6]
        print(f"Want to publish: {msg.data}")
        self.publisher.publish(msg)

    def publish_move_arm_up(self): 
        msg = Int16MultiArray()
        msg.data = [
            -1,
            12000,
            12000,
            12000,
            12000,
            12000,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T]    
        print(f"Want to publish: {msg.data}")
        self.publisher.publish(msg)

    def publish_close_gripper(self): 
        msg = Int16MultiArray()
        msg.data = [
            self.gripper_close,
            -1,
            -1,
            -1,
            -1,
            -1,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T]   
        print(f"Want to publish: {msg.data}")
        self.publisher.publish(msg)

    def publish_open_gripper(self): 
        msg = Int16MultiArray()
        msg.data = [
            self.gripper_open,
            -1,
            -1,
            -1,
            -1,
            -1,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T,
            self.T] 
        print(f"Want to publish: {msg.data}")
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MoveArmHardcoded()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
