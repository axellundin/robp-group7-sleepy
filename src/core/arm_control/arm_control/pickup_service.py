#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core_interfaces.srv import Pickup
from std_msgs.msg import Int16MultiArray
from core_interfaces.srv import YoloImageDetect
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from arm_control.arm_camera_processing import ArmCameraProcessing
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from core_interfaces.srv import MoveArm, MoveTo
import numpy as np
from _thread import start_new_thread

class PickupService(Node):
    def __init__(self):
        super().__init__('pickup_service')
        self.service = self.create_service(
            Pickup,
            'pickup',
            self.pickup_callback
        )
        
        self.arm_camera_processing = ArmCameraProcessing()
        self.joint_movement_time = 1500
        self.gripper_open_position = [3500, -1, -1, -1, -1, -1, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.gripper_close_position = [11500, -1, -1, -1, -1, -1, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.up_position = [-1, 12000, 12000, 12000, 12000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.viewing_position = [-1, 12000, 3500, 21000, 12000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.first_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.second_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 7500, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        
        self.current_target = self.up_position
        self.max_joint_diff = 500
        self.max_joint_diff_gripper = 1200
        self.bridge = CvBridge()

        self.latest_arm_image=None
        self.latest_joint_angles=None

        self.subscription_arm_image = self.create_subscription(
            Image,
            "/arm_camera/image_raw",
            self.arm_image_listener_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
       
        # Create a publisher for the arm joint angles 
        self.joint_angles_publisher = self.create_publisher(
            Int16MultiArray,
            'multi_servo_cmd_sub',
            10
        )

        group1 = ReentrantCallbackGroup()
        # Create a client to call the move arm service 
        self.move_arm_client = self.create_client(MoveArm, 'move_arm', callback_group=group1)
        while not self.move_arm_client.wait_for_service(timeout_sec=10):
            self.get_logger().warning('Move arm service not available, waiting again...')

        # Create a client to call the object detection service 
        self.yolo_client = self.create_client(YoloImageDetect, 'yolo_image_detect', callback_group=group1)
        while not self.yolo_client.wait_for_service(timeout_sec=10):
            self.get_logger().warning('YOLO service not available, waiting again...')

        self.get_logger().info('Pickup Service Started')
        
        group2 = ReentrantCallbackGroup()

        # Create subscriber on the separate node
        self.joint_angles_subscriber = self.create_subscription(
            JointState,
            'servo_pos_publisher',
            self.joint_angles_callback,
            1, 
            callback_group=group2
        )
        self.get_logger().info('Created joint subscriber')

        self.move_to_client = self.create_client(MoveTo, "MoveTo", callback_group=group2)
        while not self.move_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move to service not available, waiting again...') 

        self.arm_tucked = False

    def tuck_arm(self):
        self.get_logger().info('Tucking arm...')
        self.move_arm_to_joint_values(self.up_position)
        self.move_arm_to_joint_values(self.gripper_open_position)
        self.get_logger().info('Arm tucked')
            
    def _run_executor(self):
        try:
            self.get_logger().info('Joint executor starting to spin')
            self.joint_executor.spin()
            self.get_logger().info('Joint executor finished spinning')  # This shouldn't print unless there's an error
        except Exception as e:
            self.get_logger().error(f'Executor error: {str(e)}')

    def joint_angles_callback(self, msg):
        # self.get_logger().info('JOINT CALLBACK RECEIVED')  # Add this debug print
        self.latest_joint_angles = msg.position 
        if not self.arm_tucked:
            start_new_thread(self.tuck_arm, ())
            self.arm_tucked = True
        # Check if the joint angles have reached the viewing position 

    def arm_image_listener_callback(self, msg):
        self.latest_arm_image = msg

    def construct_pose_in_frame(self, x, y, z, theta, frame_to, frame_from=None):
        pose = PoseStamped() 
        pose.header.frame_id = frame_to 
        pose.header.stamp = self.get_clock().now().to_msg() 
        pose.pose.position.x = x 
        pose.pose.position.y = y  
        pose.pose.position.z = z  
        pose.pose.orientation.z = np.sin(theta / 2.0)
        pose.pose.orientation.w = np.cos(theta / 2.0)

        if frame_from is not None: 
            transform = self.tf_buffer.lookup_transform(frame_to, frame_from, rclpy.time.Time(seconds=0))
            pose.pose = do_transform_pose(pose.pose, transform)
            pose.header.frame_id = frame_to
        return pose

    def small_adjustment(self, x, y): 
        self.get_logger().info(f'Starting small adjustment with x={x}, y={y}')
        req_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "odom"
        goal_list.header.stamp = self.get_clock().now().to_msg()
    
        # Construct the current pose in the base link frame  
        object_in_odom = self.construct_pose_in_frame(x, y, 0.0, 0.0, "odom", "arm_base_link")
        
        # Get the latest transform from base link to odom 
        odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0)) 
        self.get_logger().info('Retrieved odom transform')

        angle_between_current_pose_and_object = np.arctan2(object_in_odom.pose.position.y - odom_transform.transform.translation.y, object_in_odom.pose.position.x - odom_transform.transform.translation.x)

        pickup_distance = 0.16 
        arm_base_link_y_shift = 0.0475  
        base_link_to_pickup_radius = (pickup_distance**2 + arm_base_link_y_shift**2)**0.5
        base_link_pickup_angle = np.arctan2(arm_base_link_y_shift, pickup_distance) 
        target_angle = angle_between_current_pose_and_object - base_link_pickup_angle  
        target_angle = target_angle
        
        object_pos = np.array([object_in_odom.pose.position.x, object_in_odom.pose.position.y])
        current_pos = np.array([odom_transform.transform.translation.x, odom_transform.transform.translation.y])
        distance_to_object = np.linalg.norm(current_pos - object_pos)
        target_pos = object_pos + (current_pos - object_pos) / distance_to_object * base_link_to_pickup_radius 

        target_pose = self.construct_pose_in_frame(target_pos[0], target_pos[1], 0.0, target_angle, "odom")

        allow_reverse = False  
        if distance_to_object < base_link_to_pickup_radius:
            allow_reverse = True 
        self.get_logger().info(f'Target_angle = {target_angle}')
        self.get_logger().info(f'Object pose: {object_in_odom.pose.position.x}, {object_in_odom.pose.position.y}')
        self.get_logger().info(f'Target pose: {target_pose}')

        # Create move to request 
        req_msg = MoveTo.Request()
        req_msg.path.poses = [target_pose]
        req_msg.max_speed = 0.2
        req_msg.allow_reverse = allow_reverse
        req_msg.max_turn_speed = 0.15
        req_msg.enforce_orientation = True 
        req_msg.stop_at_goal = True 

        self.get_logger().info('Sending move_to request...')
        future = self.move_to_client.call_async(req_msg)
        self.get_logger().info('Waiting for move_to response...')
        while not future.done(): 
            time.sleep(0.02) 
            # self.executor.spin_once(timeout_sec=0.1)
        # self.executor.spin_until_future_complete(future)
        result = future.result()
        self.get_logger().info(f'Move_to result: {result}')
        return result.success
    
    def send_yolo_request(self):
        request = YoloImageDetect.Request()
        request.camera_name = "arm_camera"
        # request.target_frame = "camera_link"
        # request.target_frame = "arm_camera"
        request.target_frame = "arm_base_link"

        self.get_logger().info(f'Sending YOLO request to {request.target_frame}')
        future = self.yolo_client.call_async(request)

        self.get_logger().info('Waiting for YOLO response...')
        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Received response: {len(response.objects)} objects detected.")

            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                category = obj.category
                print(f"Object {i + 1}:")
                print(f"  Center point: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")
                print(f"  Category: {category}")
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                prohited = ["toy_white", "toy_yellow", "box", "no_detection"]
                print(f"category = {category}, {(category in prohited)=}")
                if category in prohited:
                    continue
                print(f"  Position: {x_m} m, {y_m} m, {z_m} m")
                return True, x_m, y_m, z_m
            return False, None, None, None
        else:
            self.get_logger().error('Failed to receive response.')
            return False, None, None, None

    def move_arm_to_joint_values(self, joint_values):
        msg = Int16MultiArray()
        msg.data = joint_values
        self.joint_angles_publisher.publish(msg)
        self.get_logger().info('Moving arm to joint values...')
        latest_command = self.get_clock().now()

        while not self.check_joints(joint_values):
            if self.get_clock().now() - latest_command > rclpy.duration.Duration(seconds=5):
                # Send again 
                self.joint_angles_publisher.publish(msg)
                latest_command = self.get_clock().now()
            time.sleep(0.01)
        return True


    def check_if_object_is_reachable(self, x_m, y_m):
        # Check if the object is reachable by the arm 
        return (0.14 < x_m < 0.20 and (y_m**2 + x_m**2)**(1/2) < 0.20)

    def pickup_callback(self, request, response):
        self.get_logger().info('Starting pickup callback')
        # Reset joint angles
        self.latest_joint_angles = None
        
        # Open the gripper 
        self.move_arm_to_joint_values(self.gripper_open_position)
        self.get_logger().info('Opening gripper...')

        time.sleep(2)
        reachable = False 
        should_change_to_default_viewing_position = False
        adjusted_viewing_position = False
        # Move the arm to viewing position  
        # msg.data = self.viewing_position
        viewing_positions = [self.viewing_position, self.first_backup_viewing_position, self.second_backup_viewing_position]
        for i in range(len(viewing_positions)):
            viewing_position = viewing_positions[i]
            # self.joint_angles_publisher.publish(msg)
            self.get_logger().info(f'Trying new viewing position: {viewing_position}')
            self.move_arm_to_joint_values(viewing_position)
            time.sleep(2)

            # Perform object detection 
            if viewing_positions[i] != self.viewing_position: adjusted_viewing_position = True

            current_attempts = 0
            should_change_to_default_viewing_position = False
            should_try_new_viewing_position = False

            while not should_try_new_viewing_position:
                if adjusted_viewing_position and should_change_to_default_viewing_position:
                    self.get_logger().info('Waiting for arm to reach the viewing position...')
                    self.move_arm_to_joint_values(self.viewing_position)

                    time.sleep(2)
                
                detected, x_m, y_m, z_m = self.send_yolo_request()
                if not detected:
                    self.get_logger().error('Failed to detect object.')
                    if current_attempts > 2:
                        if viewing_position != viewing_positions[-1]:
                            self.get_logger().info('Trying new viewing position...')
                            should_try_new_viewing_position = True 
                            continue
                        response.success = False
                        return response
                    current_attempts += 1 
                    continue

                print(f'Object detected at {x_m} m, {y_m} m, {z_m} m')
                # return response
                # .------ Assume that the object is detected and the position is x_m, y_m, z_m in arm_base_link frame ------
                if self.check_if_object_is_reachable(x_m, y_m):
                    reachable = True
                    break

                self.get_logger().warning('Object is not in the correct position.')
                success = self.small_adjustment(x_m, y_m)
                should_change_to_default_viewing_position = True 
                i = 0
                current_attempts = 0

                if not success and viewing_position == viewing_positions[-1]:
                    self.get_logger().error('Failed to adjust my position.')
                    response.success = False
                    return response

            if reachable: 
                break
    
        z_m = -0.15
        # ----------------------------------------------------------------------------------

        # # 3. Calculate the gripping orientation and position 
        # # get the latest image from the arm camera 
        # while self.latest_arm_image is None:
        #     self.get_logger().info('Waiting for arm image...')
        #     time.sleep(1)
        
        # img = self.latest_arm_image
        # # process the image 
        # gripping_positions, connected_components = self.arm_camera_processing.get_gripping_position(img)
        # if len(gripping_positions) == 0:
        #     self.get_logger().error('Failed to detect gripping position.')
        #     response.success = False
        #     response.message = 'Failed to detect gripping position.'
        #     return response
        
        # Get the first gripping
        time.sleep(2)
        # 4. Move the arm to above the object 
        target = PoseStamped()
        target.header.frame_id = "arm_base_link"
        target.pose.position.x = x_m
        target.pose.position.y = y_m
        target.pose.position.z = z_m + 0.05
        rho = (x_m**2 + y_m**2)**0.5
        
        # Publish the target pose 
        self.get_logger().info('Publishing target pose...')
        request = MoveArm.Request()
        request.pose = target
        request.alpha.data =  (1.3 - 0.2) * (rho - 0.18) / (0.35 - 0.18) + 0.2
        future = self.move_arm_client.call_async(request)
        while not future.done():
            self.get_logger().info('Waiting for move arm response...')
            time.sleep(0.1)
    
        # Move the arm to the target pose 
        target.pose.position.z = z_m
        request = MoveArm.Request()
        request.pose = target
        request.alpha.data =  (1.3 - 0.2) * (rho - 0.18) / (0.35 - 0.18) + 0.2
        print(f"alpha = {request.alpha.data}") 
        time.sleep(2)
        future = self.move_arm_client.call_async(request)
        self.get_logger().info('Moving arm to target pose...')
        while not future.done():
            self.get_logger().info('Waiting for move arm response...')
            time.sleep(0.1)

        # Close the gripper 
        self.get_logger().info('Closing gripper...')
        self.move_arm_to_joint_values(self.gripper_close_position)

        # Move the arm up slightly  
        target.pose.position.z = z_m + 0.05
        request = MoveArm.Request()
        request.pose = target
        request.alpha.data = (1.3 - 0.2) * (rho - 0.18) / (0.35 - 0.18) + 0.2
        print(f"alpha = {request.alpha.data}") 
        time.sleep(2)
        future = self.move_arm_client.call_async(request)
        self.get_logger().info('Moving arm up slightly...')
        while not future.done():
            self.get_logger().info('Waiting for move arm response...')
            time.sleep(0.1)

        # Move the arm up fully 
        self.get_logger().info('Moving arm up fully...')
        self.move_arm_to_joint_values(self.up_position)

        response.success = True
        return response

    def check_joints(self, desired_joint_values):
        # Find the indicies where the joint values are not -1
        desired_joint_values = desired_joint_values[:6]
        indices = []
        for i, value in enumerate(desired_joint_values):
            if value != -1:
                indices.append(i)
        
        if self.latest_joint_angles is not None:
            joint_diff = [abs(self.latest_joint_angles[i] - desired_joint_values[i]) for i in indices]
            max_val = max(joint_diff)
            max_idx = joint_diff.index(max_val)
            if max_idx == 0:
                return max_val < self.max_joint_diff_gripper
            else:
                return max_val < self.max_joint_diff
        else: 
            self.get_logger().info('No joint angles received yet')
            return False

def main():
    rclpy.init()
    service = PickupService()
    
    try:
        ex = MultiThreadedExecutor()
        ex.add_node(service)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        service.get_logger().info('Shutting down pickup service...')
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
