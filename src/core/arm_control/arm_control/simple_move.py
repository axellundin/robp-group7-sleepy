import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import PoseStamped
from arm_control.kinematics import Kinematics
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from sensor_msgs.msg import JointState
import time
from core_interfaces.srv import MoveArm  # You'll need to create this service interface

class SimpleMove(Node):
    def __init__(self):
        super().__init__('simple_move')
        self.joint_limits = np.array([
            [-np.pi * 120 / 180, np.pi * 120 / 180], 
            [-np.pi * 60 / 180, np.pi * 90 / 180], 
            [-np.pi * 90 / 180, np.pi * 90 / 180], 
            [-np.pi * 90 / 180, np.pi * 90 / 180], 
            [-np.pi * 120 / 180, np.pi * 120 / 180]])
        self.kinematics = Kinematics(self.joint_limits)  
        
        group = ReentrantCallbackGroup()
        self.error_weight = [1/3, 1, 1, 1, 1, 1]

        # Replace subscriber with service
        self.service = self.create_service(
            MoveArm,
            'move_arm',
            self.move_arm_callback,
            callback_group=group
        )
        
        # Create publisher
        self.publisher = self.create_publisher(
            Int16MultiArray,                     # Message type
            'multi_servo_cmd_sub',            # Topic name
            10                         # QoS profile depth
        )
        self.latest_joint_angles = None

        self.joint_angles_subscriber = self.create_subscription(
            JointState,
            'servo_pos_publisher',
            self.joint_angles_callback,
            1, 
            callback_group=group
        )
        self.max_joint_diff = 1000
        self.joint_movement_time = 1000
        self.joint_velocity = 10
        self.min_joint_movement_time = 500
        
        self.get_logger().info('SimpleMove node initialized')


    def joint_angles_callback(self, msg):
        self.latest_joint_angles = msg.position

    def compute_joint_transition_time(self, desired_joint_values):
        if self.latest_joint_angles is None:
            return desired_joint_values
        current_joint_values = self.latest_joint_angles
        speeds = []
        for i in range(6):
            if desired_joint_values[i] == -1:
                speeds.append(self.joint_movement_time)
                continue
            abs_diff = abs(current_joint_values[i] - desired_joint_values[i]) 
            speeds.append(max(int(abs_diff / self.joint_velocity), self.min_joint_movement_time))

        return desired_joint_values[:6] + speeds

    def send_and_make_sure_moves_correctly(self, encoder_values):
        msg = Int16MultiArray()
        msg.data = self.compute_joint_transition_time(encoder_values)
        self.publisher.publish(msg)

        latest_command = self.get_clock().now()

        while not self.check_joints(encoder_values):
            if self.get_clock().now() - latest_command > rclpy.duration.Duration(seconds=2):
                # Send again 
                msg.data = self.compute_joint_transition_time(encoder_values)
                self.publisher.publish(msg)
                latest_command = self.get_clock().now()
            time.sleep(0.01)
        return True
    
    def check_joints(self, desired_joint_values):
        # Find the indicies where the joint values are not -1
        desired_joint_values = desired_joint_values[:6]
        indices = []
        for i, value in enumerate(desired_joint_values):
            if value != -1:
                indices.append(i)
        
        if self.latest_joint_angles is not None:
            joint_diff = [abs(self.latest_joint_angles[i] - desired_joint_values[i]) * self.error_weight[i] for i in indices]
                
            if max(joint_diff) < self.max_joint_diff:
               return True  
        return False

    def move_arm_callback(self, request, response):
        """Service callback function"""
        self.get_logger().info(f'Received: {request.pose}')
        pose = request.pose.pose
        # Extract x, y, z from the pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        # Check that the pose is reachable and does not cause self-collisions
        if max(x,-y) < 0.14 or z < -0.18 or x > 0.35 or z > 0 or (x**2 + y**2)**(1/2) > 0.35:
            self.get_logger().info('Target pose not safe, aborting')
            response.success = False
            response.message = "Target pose not safe"
            return response

        phi = float(request.phi.data)
        alpha = float(request.alpha.data)
        # Get spherical coordinates 
        try: 
            rho, z, theta = self.kinematics.cartesian_to_cylindrical(x, y, z)
            joint_angles = self.kinematics.analytical_solution(rho, z, theta, phi, alpha) 
            encoder_values = self.kinematics.manipulator_angles_to_encoders(joint_angles) 
            self.get_logger().info(f'I want to publish: {encoder_values}')
        except Exception as e:
            self.get_logger().error(f'Error during movement: {str(e)}')
            response.success = False
            response.message = f"Error during movement: {str(e)}"
            return response
        
        data = [ 
            -1,
            int(encoder_values[4]),
            int(encoder_values[3]),
            int(encoder_values[2]),
            int(encoder_values[1]),
            int(encoder_values[0]), 
            self.joint_movement_time, 
            self.joint_movement_time, 
            self.joint_movement_time, 
            self.joint_movement_time, 
            self.joint_movement_time, 
            self.joint_movement_time
        ]

        try:
            self.get_logger().info('Sending and making sure moves correctly...')
            success = self.send_and_make_sure_moves_correctly(data)
            response.success = success
            response.message = "Movement completed successfully" if success else "Movement failed"
        except Exception as e:
            response.success = False
            response.message = f"Error during movement: {str(e)}"
        self.get_logger().info('Movement completed successfully')
        return response

def main(args=None):
    rclpy.init(args=args)
    
    simple_move = SimpleMove()
    
    try: 
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(simple_move)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        simple_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
