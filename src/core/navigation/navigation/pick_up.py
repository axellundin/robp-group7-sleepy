import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from core_interfaces.srv import GridCreator
from geometry_msgs.msg import PoseStamped, Point
import tf2_geometry_msgs
from core_interfaces.srv import MoveToObject, MoveTo, YoloImageDetect

from navigation.path_planner import path_planner

from core_interfaces.srv import MoveTo
from nav_msgs.msg import Path

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from navigation.circle_creator_uppdate import circle_creator
import os
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import Marker 
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
import time
import math

class PickUp(Node):
    def __init__(self):
        super().__init__('PickUp')
        #self.get_logger().info('Pick up node started 1')

        self.collection_workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_3.tsv"
        # self.objects_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/map_3.tsv"
        self.objects_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/map_hard_collection.tsv"

        self.resolution = 5 #cm / cell
        self.padding = 25 #cm
        self.radius = 70 #cm
        self.distance_to_object_during_pickup = 10 #cm
        self.distance_to_object_during_place = 20 #cm
        self.distance_to_object = None #cm

        self.grid = None
        self.origin = None
        self.boxes = []
        self.objects = []
        self.next_object = None
        self.go_back_position = None
        self.go_back_oriorientation_z = None
        self.go_back_oriorientation_w = None
        self.latest_object_index = None
        self.planner = path_planner()
        self.doing_turn = False

        self.start_obs_grid = None
        self.local_occupancy_map = None
        self.current_waypoint = None
        self.care_about_collisions = True 
        self.local_occupancy_update_time = None
        self.should_move = True 
        self.get_logger().debug(f"\033[93m {self.should_move=} \033[0m")

        group = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()

        self.grid_fill_cli = self.create_client(GridCreator,"fill_in_workspace")

        self.camera_client = self.create_client(YoloImageDetect, 'final_detect', callback_group=group2)

        self.pickup_grid_pub = self.create_publisher(OccupancyGrid, '/pick_up_grid_map', 10)
        
        self.current_pickup_pose_pub = self.create_publisher(PoseStamped, '/current_pickup_pose', 10) 

        while not self.grid_fill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug('Grid filler service not available, waiting again...')

        self.move_client = self.create_client(MoveTo,"MoveTo")
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug('Move service not available, waiting again...')
        
        self.pickup_service = self.create_service(MoveToObject, "move_to_object", self.move_to_object_callback, callback_group=group)

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.current_object_category_pub = self.create_publisher(String, "/current_object_category", 10)

        self.local_occupancy_sub = self.create_subscription(OccupancyGrid, '/local_occupancy_map', self.local_occupancy_callback, 10, callback_group=group2)
        self.next_waypoint_sub = self.create_subscription(PoseStamped, '/next_waypoint', self.next_waypoint_callback, 10, callback_group=group2)

        # Creating buffer and transform listener
        buffer_size = rclpy.duration.Duration(seconds=10.0) 
        self.tf_buffer = Buffer(cache_time=buffer_size)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        
        self.executor = None


        # DEBUG -----
        self.collistion_check_counter = 0

    def set_executor(self, executor):
        self.executor = executor

    # callback, moves to the desired object
    def move_to_object_callback(self, request, response):
        self.moving_back = False
        self.should_move = True
        self.get_logger().debug(f"Move to object callback, {request.target=}, \033[93m {self.should_move=} \033[0m")
        if request.target == "Go BACK":
            self.moving_back = True
            #self.get_logger().info(f'Objects left before pickup: {len(self.objects)}')
            if self.latest_object_index != None:
                self.objects.pop(self.latest_object_index)
                self.latest_object_index = None
            self.publish_object_and_box_markers()
            #self.get_logger().info(f'Objects left after pickup: {len(self.objects)}')
            go_back_command = self.create_go_back_command()
            success = self.move_along_path(go_back_command)
            self.get_logger().debug(f'Move back done with result: {success}')
            response.success = success
            response.message = "Done"
            response.finished = False
            if len(self.objects) == 0:
                response.finished = True
            return response
        else:
            
            x, y = self.retrieve_object_position(request)

            should_try_again = True
            
            while should_try_again:
                self.get_logger().debug(f"Start of try again loop, \033[94m {should_try_again=} \033[0m, \033[93m {self.should_move=} \033[0m")
                should_try_again = False
                self.get_logger().info(f'Object position: x = {x} y = {y}')
                x_goal, y_goal = self.retrieve_pick_up_position(x,y) # Grid coordinates 
                self.get_logger().info(f'Pick up position: x = {x_goal} y = {y_goal}')
                angle_between_object_and_pickup = np.arctan2(y - y_goal, x - x_goal)
                start_x, start_y = self.retrieve_robot_position() # In cm 
                self.get_logger().info(f'Robot position: x = {start_x} y = {start_y}')
                start_x, start_y = self.convert_to_grid_coordinates(start_x, start_y)
                self.get_logger().info(f'Robot position (after conversion): x = {start_x} y = {start_y}')
                
                obs_grid = self.start_obs_grid.copy()
                self.insert_objects_in_map_file(obs_grid)
                self.insert_local_objects_in_map_file(obs_grid)
                self.grid = obs_grid.copy() 
                path = self.generate_path(obs_grid, start_x, start_y, x_goal, y_goal)
                #self.get_logger().info(f'Path: {path}')
                if path is None:
                    self.get_logger().debug(f"No path found, tries again, \033[94m {should_try_again=} \033[0m, \033[93m {self.should_move=} \033[0m")
                    should_try_again = True
                    continue
                move_command = self.create_move_command(path, angle_between_object_and_pickup)
                
                self.should_move = True
                self.get_logger().debug(f"\033[93m {self.should_move=} \033[0m,\033[94m {should_try_again=} \033[0m") 
                success = self.move_along_path(move_command)
                if not self.should_move:
                    should_try_again = True
                    self.get_logger().debug(f'Collision detected, trying new path!, \033[94m {should_try_again=} \033[0m, \033[93m {self.should_move=} \033[0m')
                    continue
                self.get_logger().debug(f'Move done, with result: {success}')
                if not success:
                    response.success = False
                    response.finished = False
                    response.message = "Failed to move to object"
                    return response

                # We have success, we are in the circle around the object / box.  
                # 1. Turn to face the object / box.  
                robot_x, robot_y = self.retrieve_robot_position()
                x_g, y_g = self.convert_to_real_world_coordinates(x,y) # in CM 
                request_msg = self.create_command_to_face_object(robot_x/100, robot_y/100, x_g/100, y_g/100)
                success = self.move_along_path(request_msg)
                # 2. Get a YOLO response from the RGBD camera to get a better estimate of the object / box position.  
                expected_category = self.next_object[3]
                # 3. Change the goal position to the new YOLO estimated position.   
                better_x, better_y = self.send_camera_request_and_return_closest_objxy_with_class(x_g/100,y_g/100,expected_category) # m 
                # 4. Move to the new goal position.  
                turn = self.generate_turn(better_x, better_y)

                #self.get_logger().info(f'Turn: {turn}')
                turn_command = self.create_turn_command(turn)
                #self.get_logger().info(f'Turn command: {turn_command}')
                self.should_move = True
                self.doing_turn = True
                self.get_logger().debug(f'Command to face the object created, \033[94m {should_try_again=} \033[0m, \033[93m {self.should_move=} \033[0m')
                success = self.move_along_path(turn_command)
                if not self.should_move:
                    should_try_again = True
                    self.get_logger().debug(f'Collision detected, trying new path!, \033[94m {should_try_again=} \033[0m, \033[93m {self.should_move=} \033[0m')
                    continue
                self.get_logger().debug(f'Move done, with result: {success}')
                self.doing_turn = False
                if not success:
                    response.success = False
                    response.finished = False
                    response.message = "Failed to turn to object"
                    return response

                response.success = True
                response.finished = False
                response.message = "Done"
                return response

    def next_waypoint_callback(self, msg):
        self.get_logger().debug(f'Next waypoint received: x = {msg.pose.position.x}, y = {msg.pose.position.y}') 
        self.get_logger().debug(f"States are: \033[93m {self.should_move=} \033[0m, \033[94m {self.collistion_check_counter=} \033[0m")
        pose = PoseStamped() 
        pose.header.frame_id = msg.header.frame_id 
        pose.header.stamp = msg.header.stamp  
        pose.pose = self.transform_to(msg.pose, "map", msg.header.frame_id)
        self.current_waypoint = pose 

    def insert_local_objects_in_map_file(self, grid):
        if self.local_occupancy_update_time is None: 
            return
        
        current_time = self.get_clock().now().to_msg() 
        time_since_last_update = current_time.sec + current_time.nanosec * 1e-9 - self.local_occupancy_update_time.sec - self.local_occupancy_update_time.nanosec * 1e-9
        if time_since_last_update > 10:
            return
    
        if self.local_occupancy_map is not None:
            grid[self.local_occupancy_map == -1] = -1
        
    def check_if_path_hits_obstacles(self, start_x, start_y, end_x, end_y, obstacles_grid): 
        x_min, x_max = min(start_x, end_x), max(start_x, end_x)
        y_min, y_max = min(start_y, end_y), max(start_y, end_y)
        line_len = np.sqrt((start_x - end_x)**2 + (start_y - end_y)**2)
        len_x = x_max - x_min
        len_y = y_max - y_min

        if line_len == 0: 
            return False 

        n = np.array([(end_y - start_y) / line_len, -(end_x - start_x) / line_len])
        relevant_grid = obstacles_grid[y_min:y_max+1, x_min:x_max+1]
        test_idx = np.where(np.abs(np.sum( [np.tile(np.arange(len_x+1), (len_y+1, 1)) * n[0], np.tile(np.arange(len_y+1), (len_x+1, 1)).T * n[1] ], axis=0 )) < 1 / 2) 
        values = relevant_grid[test_idx] 
        
        if len(values) == 0:
            return False
        
        return np.min(values) < 0 
    
    def local_occupancy_callback(self, msg):
        self.collistion_check_counter += 1
        self.local_occupancy_map = np.array(msg.data, dtype=np.float64).reshape((msg.info.height, msg.info.width))
        self.local_occupancy_update_time = msg.header.stamp
        
        # #self.get_logger().info(f'{(self.current_waypoint is None)=}')
        # #self.get_logger().info(f'{(not self.care_about_collisions)=}')
        # #self.get_logger().info(f'{(not self.should_move)=}')

        if (self.current_waypoint is None) or (not self.care_about_collisions) or (not self.should_move):
            return
        
        current_pose = self.transform_to(PoseStamped().pose, "map", "base_link") 
        start_x, start_y = self.convert_to_grid_coordinates(100 * current_pose.position.x, 100 * current_pose.position.y)
        end_x, end_y = self.convert_to_grid_coordinates(100 * self.current_waypoint.pose.position.x, 100 * self.current_waypoint.pose.position.y)
        collision = self.check_if_path_hits_obstacles(start_x, start_y, end_x, end_y, self.local_occupancy_map) 

        if collision and not self.moving_back:
            self.get_logger().debug(f'Collision detected, path hits obstacles')
            self.get_logger().debug(f'Current waypoint: x = {self.current_waypoint.pose.position.x}, y = {self.current_waypoint.pose.position.y}')
            self.get_logger().debug(f'Current pose: x = {current_pose.position.x}, y = {current_pose.position.y}')
            self.get_logger().debug(f'Start x: {start_x}, Start y: {start_y}, End x: {end_x}, End y: {end_y}')
            self.should_move = False
            self.get_logger().debug(f"\033[93m {self.should_move=} \033[0m")
        
    def create_go_back_command(self):
        #self.get_logger().info("create_go_back_command")
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        x, y = self.go_back_position[0], self.go_back_position[1]
        #self.get_logger().info("Go back koordinates")
        #self.get_logger().info(f"position x {x}, position y {y}")
        #self.get_logger().info(f"orientation z {self.go_back_oriorientation_z}, orientation w {self.go_back_oriorientation_w}")
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x/100
        goal_msg.pose.position.y = y/100
        goal_msg.pose.orientation.z = self.go_back_oriorientation_z
        goal_msg.pose.orientation.w = self.go_back_oriorientation_w
        goal_list.poses.append(goal_msg)
        request_msg.path = goal_list
        request_msg.max_speed = 0.3
        request_msg.max_turn_speed = 0.25
        request_msg.enforce_orientation = False
        request_msg.allow_reverse = True
        request_msg.stop_at_goal = True
        self.get_logger().debug(f"Move back command created to: x = {goal_msg.pose.position.x}, y = {goal_msg.pose.position.y}")
        return(request_msg)

    # receives the desired object, returns the position of the nearest one
    def retrieve_object_position(self,request):
        label_category_dict = { 
            "B": "box",
            "2": "ball",
            "3": "toy",
            "1": "cube"
        }
        
        #self.get_logger().info('move to object callback')
        if request.target == "BOX":
            self.distance_to_object = self.distance_to_object_during_place
            index = self.select_object(self.boxes)
            self.next_object = self.boxes[index]
            self.get_logger().debug(f'Nearest box: x = {self.next_object[0]} y = {self.next_object[1]}')
        if request.target == "OBJECT":
            self.distance_to_object = self.distance_to_object_during_pickup
            index = self.select_object(self.objects)
            self.latest_object_index = index
            self.next_object = self.objects[index]
            self.get_logger().debug(f'Next object to pick up: x = {self.next_object[0]} y = {self.next_object[1]}, ojects left = {len(self.objects)}')

        x = self.next_object[0]
        y = self.next_object[1]
        x, y = self.convert_to_grid_coordinates(x,y)
        msg = String() 
        msg.data = label_category_dict[self.next_object[3]]
        self.current_object_category_pub.publish(msg)
        return x, y
    
    def publish_marker_for_object_or_box(self, x_center, y_center, angle, category, id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        if category == "box":
            marker.ns = "boxes"
        else:
            marker.ns = "objects"
        marker.id = id
        marker.pose = PoseStamped().pose
        point = Point()
        point.x = x_center
        point.y = y_center
        point.z = 0.1
        marker.points.append(point)
        marker.pose.position.x = x_center
        marker.pose.position.y = y_center
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0

        marker.lifetime = Duration(seconds=100000000)
        if category == "cube":
            marker.color.g = 1.0
            marker.type = Marker.CUBE
        elif category == "ball":
            marker.color.r = 1.0
            marker.type = Marker.SPHERE
        elif category == "toy":
            marker.color.b = 1.0
            marker.type = Marker.CYLINDER
        elif category == "box": 
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.type = Marker.CUBE
            marker.scale.x = 0.25
            marker.scale.y = 0.15
            marker.scale.z = 0.10
        
        self.marker_pub.publish(marker)

    def publish_object_and_box_markers(self): 
        label_dict = {
            "B": "box",
            "2": "ball",
            "3": "toy",
            "1": "cube"
        }
       

        for id, object in enumerate(self.objects): 
            x_center, y_center, angle, category = object 
            self.publish_marker_for_object_or_box(x_center, y_center, angle, label_dict[category], id)

        for id, object in enumerate(self.boxes):
            x_center, y_center, angle, category = object 
            self.publish_marker_for_object_or_box(x_center, y_center, angle, label_dict[category], id)

    # returns the index of the nearest object in the list
    def select_object(self, list_of_ojects):
        current_x, current_y = self.retrieve_robot_position()
        #self.get_logger().info('selecting the object nearest to the pose of the robot')
        best_dist = 100000000
        best_object_index = None 
        #self.get_logger().info(str(list_of_ojects))
        for object_index in range(len(list_of_ojects)):
            dist_to_start = np.sqrt((current_x-list_of_ojects[object_index][0])**2+ (current_y-list_of_ojects[object_index][1])**2)
            if dist_to_start < best_dist:
                best_object_index = object_index
                best_dist = dist_to_start
    
        return best_object_index
    
    # receives the position of the object to be picked up, returns the position from where the pickup should start
    def retrieve_pick_up_position(self, x, y):
        points_on_circle = self.circle_creator.circle_filler_angle_dependent(x,y,None,self.grid,self.radius,False,True, False,False,None,None,True)
        #self.get_logger().info(f"points_on_circle before convert to real world coordinates:{points_on_circle}")
        for i in range(len(points_on_circle)):
            tmp_x, tmp_y = self.convert_to_real_world_coordinates(points_on_circle[i][0], points_on_circle[i][1])
            points_on_circle[i] = [tmp_x, tmp_y]
            #self.get_logger().info(f"{[float(tmp_x), float(tmp_y)]=}")
        #self.get_logger().info(f"points_on_circle after convert to real world coordinates:{points_on_circle}")
        position_index = self.select_object(points_on_circle)
        goal_point = points_on_circle[position_index]
        self.go_back_position = goal_point
        x_goal, y_goal = self.convert_to_grid_coordinates(goal_point[0], goal_point[1])
        return x_goal, y_goal

    def send_stop_to_move_client(self):
        request_msg = MoveTo.Request()
        request_msg.stop_at_goal = True
        request_msg.path.header.frame_id = "base_link"
        request_msg.path.header.stamp = self.get_clock().now().to_msg()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "base_link"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        request_msg.path.poses.append(goal_msg)
        future = self.move_client.call_async(request_msg)

        while not future.done():
            self.executor.spin_once(timeout_sec=0.1)

    def move_out_from_occupied(self):
        self.get_logger().debug('Tries to move out from occupied')
        unoccupied_found = False
        current_x, current_y = self.retrieve_robot_position()
        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)
        test_radius = 2*self.resolution
        test_points = [[current_x, current_y]]
        self.care_about_collisions = False
        self.should_move = True 
        while not(unoccupied_found):
            latest_path_planner_grid = self.grid.copy() 
            self.insert_objects_in_map_file(latest_path_planner_grid) 
            self.insert_local_objects_in_map_file(latest_path_planner_grid)
            test_points.extend(self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None, latest_path_planner_grid,test_radius, False, True, False,False,None,None,True))
            for point in test_points:
                if latest_path_planner_grid[point[1],point[0]] != -1:
                    safe_point = (point[0], point[1])
                    unoccupied_found = True
                    move_command = self.create_move_command([safe_point], 0)
                    move_command.enforce_orientation = False 
                    success = self.move_along_path(move_command)
                    self.get_logger().debug(f'Move out from occupied done, with result: {success}')
                    self.care_about_collisions = True
                    return
            
            test_radius += self.resolution

    # generates a path between the start and goal based on the grid map
    def generate_path(self, grid, start_x, start_y, goal_x, goal_y):
        grid_around_start = grid[start_y-5:start_y+5, start_x-5:start_x+5]
        #self.get_logger().info(f"grid size: {grid_around_start.shape}")
        self.get_logger().debug(f"Generating path from {start_x}, {start_y} to {goal_x}, {goal_y}")
        #self.get_logger().info(f"using grid around start: {grid_around_start}")
        
        grid1, path_result = self.planner.A_star(grid, [start_x, start_y], [goal_x,goal_y], 1, False, 1, True)
        self.get_logger().debug(f"Path planner done, with result: {path_result}, length of path: {len(self.planner.x_values)}")
        
        if path_result == False and not self.doing_turn:
            self.move_out_from_occupied()
            return None

        #self.publish_grid_map(grid1)
        #input("push key to generate waypoints")
        waypoints, made_it_all_the_way = self.planner.waypoint_creator(False, self.grid)
        #self.publish_grid_map(self.grid)
        self.get_logger().debug(f"Waypoints: \n: {waypoints}")
        #input("push key to plott waypoints")
        #self.get_logger().info(f"grid1: {grid1}")
        for point in waypoints:
            x,y = point
            grid1[y,x] = 50

        grid1[goal_y,goal_x] = 25
        grid1[start_y,start_x] = 75
        
        self.publish_grid_map(grid1)
        #input("push key to continue")
        #self.get_logger().info('grid map with path published')
        #self.get_logger().info(str(len(waypoints)))
        return waypoints

    # creates commands to drive along the path
    def create_move_command(self, waypoints, angle_between_object_and_pickup):
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        for point in waypoints:
            x, y = point
            #self.get_logger().info(f"Before conversion: {x}, {y}, origin: , {self.origin}")
            x, y = self.convert_to_real_world_coordinates(x,y)
            #self.get_logger().info("Waypoint koordinates (after conversion)")
            #self.get_logger().info(str(x))
            #self.get_logger().info(str(y))
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = x/100
            goal_msg.pose.position.y = y/100
            goal_msg.pose.orientation.z = 0.0 
            goal_msg.pose.orientation.w = 1.0
            goal_list.poses.append(goal_msg)
        qw = np.cos(angle_between_object_and_pickup/2) 
        qz = np.sin(angle_between_object_and_pickup/2)
        goal_list.poses[-1].pose.orientation.z = float(qz)
        goal_list.poses[-1].pose.orientation.w = float(qw)
        request_msg.path = goal_list
        request_msg.max_speed = 0.4
        request_msg.max_turn_speed = 0.25
        request_msg.enforce_orientation = True
        request_msg.allow_reverse = False
        request_msg.stop_at_goal = True
        return(request_msg)

    # generates a turn to face the object and a forward movement to approach it closely
    def generate_turn(self, goal_x=None, goal_y=None):
        if goal_x is None:
            goal_x, goal_y, goal_angle, _ = self.next_object
        else: 
            goal_x, goal_y = goal_x*100, goal_y*100
        goal_pose_base_link = PoseStamped()
        
        goal_pose_base_link.header.frame_id = "map"
        goal_pose_base_link.header.stamp = self.get_clock().now().to_msg()
        goal_pose_base_link.pose.position.x = goal_x/100
        goal_pose_base_link.pose.position.y = goal_y/100 

        self.current_pickup_pose_pub.publish(goal_pose_base_link)
        goal_pose_base_link.pose = self.transform_to_base_link(goal_pose_base_link.pose)
        return goal_pose_base_link

    def create_command_to_face_object(self, robot_x, robot_y, goal_x, goal_y): 
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        target_angle = np.arctan2(dy, dx)
        self.get_logger().warning(f"robot_x: {robot_x}, robot_y: {robot_y}, goal_x: {goal_x}, goal_y: {goal_y}")
        self.get_logger().warning(f"dx: {dx}, dy: {dy}, target_angle: {target_angle}")
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = 0
        goal_msg.pose.position.y = 0
        qw = np.cos(target_angle/2) 
        qz = np.sin(target_angle/2)
        goal_msg.pose = self.transform_to_map(goal_msg.pose)
        goal_msg.pose.orientation.z = float(qz)
        goal_msg.pose.orientation.w = float(qw)
        goal_list.poses.append(goal_msg) 
        request_msg.path = goal_list
        request_msg.max_speed = 0.2
        request_msg.max_turn_speed = 0.15
        request_msg.enforce_orientation = True
        request_msg.allow_reverse = False
        request_msg.stop_at_goal = True
        request_msg.threshold = 0.03
        return request_msg

    # creates commands to do the turn
    def create_turn_command(self, goal_pose_base_link):
        # current_x, current_y = self.retrieve_robot_position()
        # angle_to_face_object = np.arctan2( goal_pose_base_link.pose.position.y - current_y, goal_pose_base_link.pose.position.x-current_x)

        # goal_pose_camera_link = PoseStamped()
        # goal_pose_camera_link.header.frame_id = "arm_base_link"
        # goal_pose_camera_link.header.stamp = self.get_clock().now().to_msg()
        # goal_pose_camera_link.pose = self.transform_to(goal_pose_base_link.pose, "arm_base_link", "map")

        # dist = np.sqrt(goal_pose_camera_link.pose.position.x**2 + goal_pose_camera_link.pose.position.y**2)
        # angle = np.arctan2(goal_pose_camera_link.pose.position.y, goal_pose_camera_link.pose.position.x)
        # forward_move = dist - self.distance_to_object / 100
        # print("goal_pose_camera_link")
        # print(goal_pose_camera_link)
        # print("goal_pose_base_link")
        # print(goal_pose_base_link)

        # print("angle")
        # print(angle)
        # print("dist")
        # print(dist)
        # input("push key to continue")

        goal_pose_odom = PoseStamped()
        goal_pose_odom.pose = self.transform_to(goal_pose_base_link.pose, "odom", "base_link")
        #self.get_logger().info(f"goal_pose_odom = {goal_pose_odom.pose}")
        current_pose_odom = PoseStamped()
        current_pose_odom.pose = self.transform_to(current_pose_odom.pose, "odom", "base_link")
        #self.get_logger().info(f"current_pose_odom = {current_pose_odom.pose}")
        angle_between_current_pose_and_object = np.arctan2(goal_pose_odom.pose.position.y - current_pose_odom.pose.position.y, goal_pose_odom.pose.position.x - current_pose_odom.pose.position.x)

        pickup_distance = 0.14
        arm_base_link_y_shift = 0.0475  
        base_link_to_pickup_radius = (pickup_distance**2 + arm_base_link_y_shift**2)**0.5
        base_link_pickup_angle = np.arctan2(arm_base_link_y_shift, pickup_distance) 
        target_angle = angle_between_current_pose_and_object + base_link_pickup_angle 
        #self.get_logger().info(f"target_angle = {target_angle}")

        object_pos = np.array([goal_pose_base_link.pose.position.x, goal_pose_base_link.pose.position.y])
        distance_to_object = np.linalg.norm(object_pos)
        target_pos = object_pos - object_pos / distance_to_object * base_link_to_pickup_radius 

        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_pos[0]
        goal_msg.pose.position.y = target_pos[1]
        qw = np.cos(target_angle/2) 
        qz = np.sin(target_angle/2)
        # goal_msg.pose.position.x = forward_move * np.cos(angle)
        # goal_msg.pose.position.y = forward_move * np.sin(angle)
        # qw = np.cos(angle_to_face_object/2) 
        # qz = np.sin(angle_to_face_object/2)
        self.go_back_oriorientation_z = float(qz)
        self.go_back_oriorientation_w = float(qw)
        #self.get_logger().info(f'Before transform: {goal_msg.pose}')
        goal_msg.pose = self.transform_to_map(goal_msg.pose)
        #goal_msg.pose = self.transform_to(goal_msg.pose, "map", "arm_base_link")
        goal_msg.pose.orientation.z = float(qz)
        goal_msg.pose.orientation.w = float(qw)
        #self.get_logger().info(f'After transform: {goal_msg.pose}')
        goal_list.poses.append(goal_msg)
        request_msg.path = goal_list
        request_msg.max_speed = 0.2
        request_msg.max_turn_speed = 0.15
        request_msg.enforce_orientation = True
        request_msg.allow_reverse = False
        request_msg.stop_at_goal = True
        request_msg.threshold = 0.03
        return request_msg
    
    def move_callback(self, future):
        self.done_move = True

    def turn_callback(self, future):
        self.done_turn = True
        
    # sends drive commands to move the robot
    def move_along_path(self,move_command):
        self.get_logger().debug(f"Moving along: {[ (p.pose.position.x, p.pose.position.y) for p in move_command.path.poses ]}")

        future = self.move_client.call_async(move_command)

        #self.get_logger().info('Waiting for move goal to be done')

        # self.executor.spin_until_future_complete(future)
        while ( not future.done() ) and self.should_move:
            self.executor.spin_once(timeout_sec=0.1)
        # future.add_done_callback(self.move_callback) 
        if not self.should_move:
            self.get_logger().debug(f'Move goal cancelled due to collision detection, \033[93m {self.should_move=} \033[0m')
            # Send stop to move to
            self.send_stop_to_move_client()
            return False
        
        #self.get_logger().info('Move goal done')
        result = future.result() 
        return result.success

    def transform_to(self, goal_pose, target_frame, source_frame):
        time = rclpy.time.Time(seconds=0)

        try:
            t = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            time,
            )
        except TransformException as ex:
            self.get_logger().debug(
                f'Could not transform {"map"} to {"base_link"}: {ex}')
            return
        transformed_pose= tf2_geometry_msgs.do_transform_pose(goal_pose, t)
        return transformed_pose

    def transform_to_map(self, goal_pose):
        return self.transform_to(goal_pose, "map", "base_link")

    # transformes goal pose to base link
    def transform_to_base_link(self, goal_pose):
        return self.transform_to(goal_pose, "base_link", "map")
        
    # retrieve robot position
    def retrieve_robot_position(self):
        #  rclpy.spin_once(self, timeout_sec=.5)
        time = rclpy.time.Time(seconds=0)

        try:
            t = self.tf_buffer.lookup_transform(
            "map",
            "base_link",
            time,
            )
            #self.get_logger().info(f'Retrieved transform: x={t.transform.translation.x}, y={t.transform.translation.y}')
        except TransformException as ex:
            self.get_logger().debug(
                f'Could not transform {"map"} to {"base_link"}: {ex}')
            return
        return t.transform.translation.x*100, t.transform.translation.y*100

    
    def convert_to_grid_coordinates(self, x, y):
        return int(np.round((x/self.resolution)+self.origin[0],0)), int(np.round((y/self.resolution)+self.origin[1],0))

    def convert_to_real_world_coordinates(self, x, y):
        return (x - self.origin[0])*self.resolution, (y - self.origin[1])*self.resolution

    # places the objects in the grid map before publishing
    def insert_objects_in_map_file(self, grid):
        #self.get_logger().info(f'inserting objects in grid map')
        grid2 = grid.copy()
        for object in self.objects:
            x, y = self.convert_to_grid_coordinates(object[0],object[1])
            fill_in_cells = self.circle_creator.circle_filler_angle_dependent(x,y,None,grid2,self.padding+7,False,False, False,True,None,None,False)
            for cell in fill_in_cells:
                grid[cell[1],cell[0]] = -1
            grid[y,x] = 90
        half_diagonal_box = np.sqrt((24.5/2)**2+(15.5/2)**2)
        for box in self.boxes:
            x, y = self.convert_to_grid_coordinates(box[0],box[1])
            fill_in_cells = self.circle_creator.circle_filler_angle_dependent(x,y,None,grid2,half_diagonal_box+self.padding+7,False,False, False,True,None,None,False)
            for cell in fill_in_cells:
                grid[cell[1],cell[0]] = -1
            grid[y,x] = 90
            # x_max, y_max = self.convert_to_grid_coordinates(box[0]+half_diagonal_box+self.padding,box[1]+half_diagonal_box+self.padding)
            # x_min, y_min = self.convert_to_grid_coordinates(box[0]-half_diagonal_box-self.padding,box[1]-half_diagonal_box-self.padding)
            # x_max = int(np.min([int(grid.shape[1])-1, x_max]))
            # y_max = int(np.min([int(grid.shape[0])-1, y_max]))
            # x_min = int(np.max([0, x_min]))
            # y_min = int(np.max([0, y_min]))
            # for cell_x in range(x_min,x_max+1):
            #     for cell_y in range(y_min,y_max+1):
            #         grid[cell_y, cell_x] = -1
            mid_box_x, mid_box_y = self.convert_to_grid_coordinates(box[0],box[1])
            grid[mid_box_y,mid_box_x] = 90

        #self.get_logger().info(f'all objects inserted in grid map')
    
    # publishes the grid map.
    def publish_grid_map(self, grid):
        #self.get_logger().info(f'publishing grid map')
        
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" 

        msg.info.resolution = float(self.resolution/100)
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]

        msg.info.origin.position.x = -float(self.origin[0]/100)*self.resolution
        msg.info.origin.position.y = -float(self.origin[1]/100)*self.resolution
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  
        grid = np.round(grid,0).astype(np.int8)
        msg.data = grid.flatten().tolist()
        
        self.pickup_grid_pub.publish(msg)
        #self.get_logger().info(f'grid map published')

    # retrieve objects from file
    def retrieve_objects_from_file(self):
        #self.get_logger().info('retrieve objects from file')
        file = self.objects_file
        read_start_row = 0 
        file_path = os.path.expanduser(file)
        while True:
            try:
                data = np.loadtxt(file_path, delimiter="\t", dtype = "str")
                break
            except ValueError as e:
                read_start_row += 1
        #self.get_logger().info(str(data))
        for object in data:
            if object[0] == 'B':
                self.boxes.append([float(object[1]),float(object[2]), float(object[3]),object[0]])
            else:
                self.objects.append([float(object[1]),float(object[2]), float(object[3]),object[0]])
        self.publish_object_and_box_markers()
        self.get_logger().debug(f'Objects retrieved from file: {len(self.boxes)} boxes, {len(self.objects)} other objects')
        self.start_obs_grid = self.grid.copy()
        pub_map = self.grid.copy()
        self.insert_objects_in_map_file(pub_map)
        
        self.publish_grid_map(pub_map)

    # fetch grid map
    def fetch_map(self):
        self.get_logger().debug(f'Fetching map, with: {self.padding=}, {self.resolution=}')
        req = GridCreator.Request()
        req.padding = self.padding
        req.resolution = self.resolution
        req.file_name = self.collection_workspace_file
        future = self.grid_fill_cli.call_async(req)
        self.executor.spin_until_future_complete(future)

        flat_grid = future.result()
        self.origin = [flat_grid.grid.info.origin.position.x, flat_grid.grid.info.origin.position.y]
        self.grid = np.array(flat_grid.grid.data,dtype=np.float64).reshape((flat_grid.grid.info.height, flat_grid.grid.info.width))
        
        #self.get_logger().info('map fetched')
        self.circle_creator = circle_creator(self.resolution)

    # call camera and find obj x y 
    def send_camera_request_and_return_closest_objxy_with_class(self, x, y, obj_class):
        label_category_dict = { 
            "B": "box",
            "2": "ball",
            "3": "toy",
            "1": "cube"
        }
        obj_class = label_category_dict[obj_class]

        request = YoloImageDetect.Request()
        request.camera_name = "rgbd_camera"
        request.target_frame = "map"

        future = self.camera_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        while not future.done():
            self.executor.spin_once(timeout_sec=0.1) 

        if future.result() is not None:
            response = future.result()
            closest_distance = 1
            find_aligned_obj = False
            rx, ry = -1, -1
            self.get_logger().warning(f"x: {x}, y: {y}, class: {obj_class}")
            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                x_real = (pose.pose.position.x)
                y_real = (pose.pose.position.y)
                category = obj.category
                self.get_logger().warning(f"real x: {x_real}, real y: {y_real}, category: {category}")
                if category != "no_detection":
                    self.get_logger().warning("trying to make sure accurate obj position, but no obj detected")
                    return None, None
                print(f"Object {i + 1}:")
                if category == obj_class:
                    distance = math.sqrt((x_real - x)**2 + (y_real - y)**2)
                    if distance < closest_distance:
                        closest_distance = distance
                        find_aligned_obj = True
                        rx, ry = x_real, y_real

            if find_aligned_obj == True:
                self.get_logger().info("successfully find predicted obj")
                return rx, ry
            else:
                self.get_logger().warning('Failed to find obj with predicted class.')
                return None, None
                
        else:
            self.get_logger().error('Failed to receive response from camera.')
            return None, None

def main():
    rclpy.init()
    node = PickUp()
    executor = MultiThreadedExecutor()

    try:
        node.set_executor(executor)
        executor.add_node(node)
        node.fetch_map()
        node.retrieve_objects_from_file()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.alive = False
        # Proper cleanup
        node.get_logger().warning("shutting down")
        executor.shutdown(timeout_sec=1.0)  # Give time for cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()