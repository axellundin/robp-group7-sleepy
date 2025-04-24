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
from arm_control.arm_camera_processing import ArmCameraProcessing, BoxPositionConverter
from arm_control.box_position_pub import quaternion_to_matrix
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from core_interfaces.srv import MoveArm, MoveTo
import numpy as np
from _thread import start_new_thread
import cv2
from std_msgs.msg import String


class PickupService(Node):
    def __init__(self):
        super().__init__('pickup_service')
        self.service = self.create_service(
            Pickup,
            'pickup',
            self.pickup_callback
        )
        
        self.arm_camera_processing = ArmCameraProcessing()
        self.joint_movement_time = 1000
        self.min_joint_movement_time = 500
        self.joint_velocity = 10
        self.gripper_open_position = [3500, -1, -1, -1, -1, -1, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.gripper_close_position = [11500, -1, -1, -1, -1, -1, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.up_position = [-1, 12000, 12000, 12000, 12000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.viewing_position = [-1, 12000, 3500, 21000, 12000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.first_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 12000, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        self.second_backup_viewing_position = [-1, 12000, 5000, 21000, 14000, 7500, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time, self.joint_movement_time]
        

        self.current_target = self.up_position
        self.max_joint_diff = 800
        self.max_joint_diff_gripper = 2000
        self.bridge = CvBridge()

        self.box_position_converter = BoxPositionConverter()

        self.latest_arm_image=None
        self.latest_joint_angles=None

        self.subscription_arm_image = self.create_subscription(
            Image,
            "/arm_camera/image_raw",
            self.arm_image_listener_callback,
            10
        )

        # Create publisher for processed images
        self.grip_publisher = self.create_publisher(
            Image,
            '/gripping',
            10
        )

        self.current_object_category = None
        self.subscription_current_object_category = self.create_subscription(
            String,
            "/current_object_category",
            self.current_object_category_callback,
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

    def current_object_category_callback(self, msg):
        self.current_object_category = msg.data

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
    
    def filter_detections(self, detections):
        if self.current_object_category is None:
            return [detection for detection in detections if detection.category not in ["box", "no_detection"]]
        
        return [detection for detection in detections if detection.category == self.current_object_category]

    def small_adjustment(self, x, y): 
        self.get_logger().info(f'Starting small adjustment with x={x}, y={y}')
        req_msg = MoveTo.Request()
        goal_list = Path()
        req_msg.threshold = 0.01
        goal_list.header.frame_id = "odom"
        goal_list.header.stamp = self.get_clock().now().to_msg()
    
        # Construct the current pose in the base link frame  
        object_in_odom = self.construct_pose_in_frame(x, y, 0.0, 0.0, "odom", "arm_base_link")
        
        # Get the latest transform from base link to odom 
        odom_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time(seconds=0)) 
        self.get_logger().info('Retrieved odom transform')

        angle_between_current_pose_and_object = np.arctan2(object_in_odom.pose.position.y - odom_transform.transform.translation.y, object_in_odom.pose.position.x - odom_transform.transform.translation.x)

        pickup_distance = 0.16 
        arm_base_link_y_shift = -0.0475  
        base_link_to_pickup_radius = (pickup_distance**2 + arm_base_link_y_shift**2)**0.5
        base_link_pickup_angle = np.arctan2(arm_base_link_y_shift, pickup_distance) 
        target_angle = angle_between_current_pose_and_object - base_link_pickup_angle  
        
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

            # This is for debugging purposes
            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                category = obj.category
                print(f"Object {i + 1}:")
                print(f"  Center point: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")
                print(f"  Category: {category}")
                x_m = (pose.pose.position.x)
                y_m = (pose.pose.position.y)
                z_m = (pose.pose.position.z)
                print(f"  Position: {x_m} m, {y_m} m, {z_m} m")

            filtered_detections = self.filter_detections(response.objects)
            if len(filtered_detections) == 0:
                return False, None, None, None, None, None, None, None
            obj = filtered_detections[0]
            return True, obj.center_point.pose.position.x, obj.center_point.pose.position.y, obj.center_point.pose.position.z, obj.category, obj.image, obj.topleft_point.pose.position.x, obj.topleft_point.pose.position.y
        else:
            self.get_logger().error('Failed to receive response.')
            return False, None, None, None, None, None, None, None

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

    def move_arm_to_joint_values(self, joint_values):
        msg = Int16MultiArray()
        msg.data = self.compute_joint_transition_time(joint_values)
        self.get_logger().info(f'Moving arm to joint values: {msg.data}')
        self.joint_angles_publisher.publish(msg)
        self.get_logger().info('Moving arm to joint values...')
        latest_command = self.get_clock().now()

        while not self.check_joints(joint_values):
            if self.get_clock().now() - latest_command > rclpy.duration.Duration(seconds=1):
                # Send again 
                msg.data = self.compute_joint_transition_time(joint_values)
                self.get_logger().info(f'Moving arm to joint values: {msg.data}')
                self.joint_angles_publisher.publish(msg)
                latest_command = self.get_clock().now()
            time.sleep(0.01)
        return True

    def find_gripping_angle_and_center(self, image, upper_left_y):
        image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        threshold = min(image.shape[1], 400 - upper_left_y)
        # Separate the image into the color channels: 
        b, g, r = cv2.split(image)

        thresh_g = cv2.Canny(g, 120, 200)
        thresh_b = cv2.Canny(b, 120, 200)
        thresh_r = cv2.Canny(r, 120, 200)

        contours_g, hierarchy_g = cv2.findContours(thresh_g, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_b, hierarchy_b = cv2.findContours(thresh_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_r, hierarchy_r = cv2.findContours(thresh_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # thresh = cv2.Canny(cv_image, 150, 200)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = contours_g + contours_b + contours_r
        contours = [contour for contour in contours if contour[:,0,1].max() < threshold]
        # thresh = cv2.Canny(image, 100, 200)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get the minimum area rectangle   
        contours = [contour for contour in contours if len(contour) > 100]
        cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
        com = self.compute_center_of_mass_from_contours(contours, image)
        # Convert all contours to convex hull of the union of all contours 
        contours = [cv2.convexHull(np.concatenate(contours)) for contour in contours]

        # Draw contour on image: 
        cv2.drawContours(image, contours, -1, (0, 0, 255), 2)

        # Get the minimum area rectangle    
        max_contour = max(contours, key=cv2.contourArea)
        center,size,angle = cv2.minAreaRect(max_contour)
        box = cv2.boxPoints((center,size,angle))
        box = np.intp(box)
        cv2.drawContours(image, [box], 0, (255, 0, 0), 1)

        w,h = size
        min_angle = 0
        max_angle = 0
        if w > h:
            min_angle = angle 
            max_angle = angle + 90
        else:
            min_angle = angle + 90
            max_angle = angle 

        min_angle = min_angle * np.pi / 180
        max_angle = max_angle * np.pi / 180

        # Wrap to -pi/2 to pi/2:
        min_angle = np.arctan2(np.sin(min_angle), np.cos(min_angle))
        max_angle = np.arctan2(np.sin(max_angle), np.cos(max_angle))
        min_angle = - ((min_angle + np.pi/2) % np.pi - np.pi/2)
        max_angle = - ((max_angle + np.pi/2) % np.pi - np.pi/2)    

        cv2.circle(image, com, 5, (0, 0, 255), -1)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                # ros_image = self.bridge.cv2_to_imgmsg(thresh, encoding='8UC1')
        self.grip_publisher.publish(ros_image)
            

        return min_angle, max_angle, com

    def check_if_object_is_reachable(self, x_m, y_m):
        # Check if the object is reachable by the arm 
        return (0.14 < x_m < 0.20 and (y_m**2 + x_m**2)**(1/2) < 0.20) and (-0.8 < y_m < 0.8)

    def compute_center_of_mass_from_contours(self, contours, cv_image): 

        # 1. Overlay all contours 
        overlay = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=np.uint8) 
        cv2.drawContours(overlay, contours, -1, (255, 255, 255), 1)
        # 2. Compute the center of mass of the overlayed contours 
        M = cv2.moments(overlay)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        # 3. Return the center of mass 
        return cx, cy

    def get_object_position_from_image_pixels(self, x,y):
        try:
            transform_arm_base_link_to_arm_camera = self.tf_buffer.lookup_transform('arm_base_link', 'arm_camera', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"cannot transform to arm_base_link from arm_camera: {e}")
            return None, -1
        # Get matrix of whole transform
        # Extract translation and rotation from transform
        translation = transform_arm_base_link_to_arm_camera.transform.translation
        rotation = transform_arm_base_link_to_arm_camera.transform.rotation
        
        # Convert quaternion to rotation matrix
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        transform_matrix = quaternion_to_matrix(quat)
        
        # Add translation to transformation matrix
        transform_matrix[0, 3] = translation.x
        transform_matrix[1, 3] = translation.y
        transform_matrix[2, 3] = translation.z

        return self.box_position_converter.convert_image_to_world_coordinates(transform_matrix, x, y, -0.16) 
    
    def check_if_object_is_still_there(self, expected_category, expected_upper_left_x, expected_upper_left_y, pos_tol=10): 
        result = False
        for i in range(3):
            self.get_logger().info(f'Checking if object is still there (attempt {i+1})')
            request = YoloImageDetect.Request()
            request.camera_name = "arm_camera"
            request.target_frame = "arm_base_link"

            self.get_logger().info(f'Sending YOLO request to {request.target_frame}')
            future = self.yolo_client.call_async(request)

            self.get_logger().info('Waiting for YOLO response...')
            while not future.done():
                time.sleep(0.1)

            if future.result() is None: 
                self.get_logger().error('Failed to receive response.')
                result = False
                continue
            
            response = future.result()
            self.get_logger().info(f"Received response: {len(response.objects)} objects detected.")

            filtered_detections = self.filter_detections(response.objects)
            
            for detection in filtered_detections: 
                if detection.category == expected_category:
                    self.get_logger().info(f'Object is still there. upper left x = {detection.topleft_point.pose.position.x}, upper left y = {detection.topleft_point.pose.position.y}')
                    time.sleep(10)
                    max_deviation = max(abs(detection.topleft_point.pose.position.x - expected_upper_left_x), abs(detection.topleft_point.pose.position.y - expected_upper_left_y))
                    if max_deviation < pos_tol:
                        return True
                    self.get_logger().info(f'Object is still there but too far away from expected position. max deviation = {max_deviation}')
            result = False 
        
        return result
    
    def pickup_callback(self, request, response):
        self.get_logger().info('Starting pickup callback')
        # Reset joint angles
        while True: 
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
            category = None
            image = None
            top_x, top_y = None, None
            x_m, y_m = None, None
            test_pos_index = 0
            while True:
                while test_pos_index < len(viewing_positions):
                    viewing_position = viewing_positions[test_pos_index]
                    # self.joint_angles_publisher.publish(msg)
                    self.get_logger().info(f'Trying new viewing position: {viewing_position}')
                    self.move_arm_to_joint_values(viewing_position)
                    time.sleep(2)

                    # Perform object detection 
                    if viewing_positions[test_pos_index] != self.viewing_position: adjusted_viewing_position = True

                    current_attempts = 0
                    should_change_to_default_viewing_position = False
                    should_try_new_viewing_position = False

                    while not should_try_new_viewing_position:
                        if adjusted_viewing_position and should_change_to_default_viewing_position:
                            self.get_logger().info('Waiting for arm to reach the viewing position...')
                            self.move_arm_to_joint_values(self.viewing_position)

                            time.sleep(2)
                        
                        detected, x_yolo, y_yolo, _, category, image, top_x, top_y = self.send_yolo_request()
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

                        print(f'Object detected at {x_yolo} m, {y_yolo} m')
                        # return response
                        # .------ Assume that the object is detected and the position is x_m, y_m, z_m in arm_base_link frame ------
                        if self.check_if_object_is_reachable(x_yolo, y_yolo):
                            reachable = True
                            break

                        self.get_logger().warning('Object is not in the correct position.')
                        success = self.small_adjustment(x_yolo, y_yolo)
                        should_change_to_default_viewing_position = True 
                        test_pos_index = 0
                        current_attempts = 0

                        if not success and viewing_position == viewing_positions[-1]:
                            self.get_logger().error('Failed to adjust my position.')
                            response.success = False
                            return response

                    test_pos_index += 1

                    if reachable: 
                        break
            
                z_m = -0.15
                # ----------------------------------------------------------------------------------
                
                x_m, y_m = x_yolo, y_yolo
                z_headroom = 0.05

                phi = 0
                if category != "ball":
                    try:
                        min_angle, max_angle, center = self.find_gripping_angle_and_center(image, top_y)
                    except Exception as e:
                        self.get_logger().error(f'Error finding gripping angle and center: {e}')
                        continue
                    if category == "cube":
                        phi = min(min_angle, max_angle, key=lambda x: abs(x))
                    elif category == "toy":
                        phi = min_angle 
                        z_headroom = 0.08
                        print(f'Center: {center}, top_x: {top_x}, top_y: {top_y}')
                        x_m, y_m, _ = self.get_object_position_from_image_pixels(top_x + center[0], top_y + center[1])
                break

            if not self.check_if_object_is_reachable(x_m, y_m): 
                x_m, y_m = x_yolo, y_yolo
            
            # Get the first gripping
            time.sleep(1)
            # 4. Move the arm to above the object 
            target = PoseStamped()
            target.header.frame_id = "arm_base_link"
            target.pose.position.x = x_m
            target.pose.position.y = y_m
            target.pose.position.z = z_m + z_headroom
            rho = (x_m**2 + y_m**2)**0.5

            # Publish the target pose 
            self.get_logger().info('Publishing target pose...')
            request = MoveArm.Request()
            request.pose = target
            request.alpha.data =  (1.3 - 0.2) * (rho - 0.18) / (0.35 - 0.18) + 0.2
            request.phi.data = float(phi)
            future = self.move_arm_client.call_async(request)
            while not future.done():
                self.get_logger().info('Waiting for move arm response...')
                time.sleep(0.1)
        
            # Move the arm to the target pose 
            target.pose.position.z = z_m
            request = MoveArm.Request()
            request.pose = target
            request.alpha.data =  (1.3 - 0.2) * (rho - 0.18) / (0.35 - 0.18) + 0.2
            request.phi.data = float(phi)
            print(f"alpha = {request.alpha.data}") 
            time.sleep(2)
            future = self.move_arm_client.call_async(request)
            self.get_logger().info('Moving arm to target pose...')
            while not future.done():
                self.get_logger().info('Waiting for move arm response...')
                time.sleep(0.1)
            
            time.sleep(1)
            # Close the gripper 
            self.get_logger().info('Closing gripper...')
            self.move_arm_to_joint_values(self.gripper_close_position)

            # Move the arm up slightly  
            target.pose.position.z = z_m + 0.05 
            request = MoveArm.Request()
            request.pose = target
            request.alpha.data = (1.3 - 0.2) * (rho - 0.18) / (0.35 - 0.18) + 0.2
            request.phi.data = float(phi)
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

            # # Check if the object is still there 
            # time.sleep(2)
            # if not self.check_if_object_is_still_there(category, 111, 260, pos_tol=200): 
            #     self.get_logger().error('I think I have dropped the object.')
            #     continue

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
