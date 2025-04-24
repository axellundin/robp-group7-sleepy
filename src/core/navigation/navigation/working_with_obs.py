import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from core_interfaces.srv import MoveTo
from core_interfaces.srv import GridCreator
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from navigation.path_planner import path_planner
from navigation.circle_creator_uppdate import circle_creator
from core_interfaces.srv import YoloImageDetect
from nav_msgs.msg import Path
from math import cos, sin, atan2, fabs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time as std_time
import random
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Explorer_node(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info('explorer node started 3')

        self.exploration_workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_3.tsv"
        self.resolution = 5 #cm / cell
        self.padding = 28 #cm
        self.max_detection_range = 100 #cm
        self.max_detection_range_grid_coordinates = np.round((self.max_detection_range)/self.resolution, 0)
        self.optimal_detection_distance = 70 #cm
        self.camera_angle = np.pi/3
        self.approximate_area_size = 130 #cm
        self.accepted_exploration_ratio_per_area = 0.9

        self.full_rotation_limit = 0.38
        self.nr_tries_full_rotation = 50
        self.nr_tries_full_area = 30
        self.minimum_accepted_exploration_ratio_full_rotation = 0.8
        self.full_rotation_reduction_accepted_exploration_ratio = (self.minimum_accepted_exploration_ratio_full_rotation/self.accepted_exploration_ratio_per_area)**(1/(self.nr_tries_full_rotation-self.nr_tries_full_area))

        self.start_time_generate_point = None
        self.time_look_at_point_dist_rejected = 0
        self.time_look_at_point_yaw_rejected = 0
        self.time_look_at_point_rejected_other_reason = 0
        self.time_attempt_detect_from_current_position = 0
        self.time_look_from_point_dist_rejected = 0
        self.time_look_from_point_yaw_rejected = 0
        self.time_to_high_claim = 0

        self.tot_rejected_points = 0

        # distance requirement to the look-at point
        self.look_at_points_nr_dist_rejected_threshold = 10000 # itterations before increment accepted dist
        self.look_at_points_nr_dist_rejected = 0
        self.tot_look_at_points_nr_dist_rejected = 0
        self.look_at_points_dist_threshold_start = self.max_detection_range_grid_coordinates
        self.look_at_points_dist_threshold_new_area = 10000#self.max_detection_range_grid_coordinates + np.round((self.approximate_area_size/2)/self.resolution,0)
        self.increment_threshold_dist_look_at_point = np.round((self.approximate_area_size/20)/self.resolution,0) # cells
        self.look_at_points_dist_threshold = self.look_at_points_dist_threshold_start + self.increment_threshold_dist_look_at_point

        # yaw change requirement to the look-at point
        self.look_at_points_nr_yaw_rejected = 0
        self.look_at_points_yaw_threshold_start = self.camera_angle/2
        self.increment_threshold_yaw_look_at_point = 0.0005 #rad
        self.look_at_points_yaw_threshold = self.look_at_points_yaw_threshold_start

        # requirement on how many times the robot should attempt to detect sufficiently from its current position
        self.attempt_detect_from_current_position_threshold_start = 24
        self.attempt_detect_from_current_position_threshold_new_area = 0
        self.attempt_detect_from_current_position_threshold = 0
        self.attempt_detect_from_current_position = 0

        # distance requirement to the look-from point
        self.look_from_points_nr_dist_rejected_threshold = 10 # itterations before increment accepted dist
        self.tot_look_from_points_nr_dist_rejected = 0
        self.look_from_points_nr_dist_rejected = 0
        self.increment_threshold_dist_look_from_point = np.round((self.approximate_area_size/10)/self.resolution,0) # cells
        self.look_from_points_extra_dist_threshold = self.increment_threshold_dist_look_from_point
        self.look_from_points_dist_threshold = None

        # yaw requirement to the look-from point
        self.look_from_points_nr_yaw_rejected_threshold = 20 # itterations before increment accepted dist
        self.tot_look_from_points_nr_yaw_rejected = 0
        self.look_from_points_nr_yaw_rejected = 0
        self.look_from_points_yaw_threshold_new_area = 10000
        self.increment_threshold_yaw_look_from_point = self.camera_angle/2
        self.look_from_points_yaw_threshold = self.increment_threshold_yaw_look_from_point

        # exploration ratio requirement
        self.nr_to_high_claim_threshold = 3 # itterations before reduce accepted exploration ratio
        self.nr_to_high_claim_rejected = 0
        self.tot_nr_to_high_claim_rejected = 0
        self.reduction_accepted_exploration_ratio  = 0.8 # cells
        
        approximate_circular_area_covering_entire_area = ((np.sqrt(2)*self.approximate_area_size/2)**2)*np.pi
        circular_area_covered_by_camera = np.pi*self.max_detection_range**2
        self.start_accepted_exploration_ratio = np.min([1.0,0.5*((circular_area_covered_by_camera/approximate_circular_area_covering_entire_area)* ((self.camera_angle)/(2*np.pi)))])
        self.accepted_exploration_ratio = None

        self.angle_between_test_look_from_points = np.pi/6 #rad
        self.nr_of_test_look_from_points = int(np.round((2*np.pi)/self.angle_between_test_look_from_points,0))

        self.origin = None 

        self.detected_objects = []
 
        self.areas = []

        self.status = ""
        
        group2 = ReentrantCallbackGroup()
        group3 = ReentrantCallbackGroup()
        group4 = ReentrantCallbackGroup()

        self.explorer_grid = None
        self.path_planner_grid = None
        self.safe_space_grid = None

        # creating publisher for path grid, exploration grid and area division
        self.path_grid_pub = self.create_publisher(OccupancyGrid, '/path_grid_map', 10, callback_group=group2)
        self.explore_grid_pub = self.create_publisher(OccupancyGrid, '/explore_grid_map', 10, callback_group=group3)
        self.area_grid_pub = self.create_publisher(OccupancyGrid, '/area_grid_map', 10, callback_group=group4)
        self.local_occupancy_map_sub = self.create_subscription(OccupancyGrid, '/local_occupancy_map', self.local_occupancy_map_callback, 10, callback_group=group4)
        # creating grid map client
        self.grid_fill_cli = self.create_client(GridCreator,"fill_in_workspace")
        while not self.grid_fill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('grid filler service not available, waiting again...')

        # creating camera client
        self.move_client = self.create_client(MoveTo,"MoveTo", callback_group=group4)
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move service not available, waiting again...')
        
        # creating camera client
        self.camera_client = self.create_client(YoloImageDetect, 'yolo_image_detect', callback_group=group2)
        self.object_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        while not self.camera_client.wait_for_service(timeout_sec=10):
            self.get_logger().info('Service not available, waiting again...')
        
        # creating buffer and transform listener
        buffer_size = rclpy.duration.Duration(seconds=10.0) 
        self.tf_buffer = Buffer(cache_time=buffer_size)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        std_time.sleep(1)

    def fetch_map(self, padding):
        self.get_logger().info('fetching map')
        req = GridCreator.Request()
        req.padding = padding
        req.resolution = self.resolution
        req.file_name = self.exploration_workspace_file
        future = self.grid_fill_cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future)

        # executor = SingleThreadedExecutor() 
        # executor.add_node(self) 
        while not future.done():
            self.executor.spin_once(timeout_sec=0.01)
        flat_grid = future.result()
        self.origin = [flat_grid.grid.info.origin.position.x, flat_grid.grid.info.origin.position.y]
        grid = np.array(flat_grid.grid.data,dtype=np.float64).reshape((flat_grid.grid.info.height, flat_grid.grid.info.width))
        return grid
        
    def local_occupancy_map_callback(self, msg):
        self.local_occupancy_map = np.array(msg.data, dtype=np.float64).reshape((msg.info.height, msg.info.width))

    # Initial: divide the grid map into areas
    def divide_grid_areas(self):
        self.get_logger().info('dividing the grid map into areas')
        hight = self.explorer_grid.shape[0]*self.resolution
        width = self.explorer_grid.shape[1]*self.resolution
        hight_divider = hight//self.approximate_area_size
        width_divider = width//self.approximate_area_size
        area_hight = int((hight/hight_divider)/self.resolution)
        area_width = int((width/width_divider)/self.resolution)
        for row in range(hight_divider):
            for column in range(width_divider):

                column_end_point = int(np.ceil((column+1)*area_width))
                if column_end_point >= width/self.resolution:
                    column_end_point = int(np.ceil(width/self.resolution))-1

                row_end_point = int(np.ceil((row+1)*area_hight))
                if row_end_point >= hight/self.resolution:
                    row_end_point = int(np.ceil(hight/self.resolution))-1

                midpoint_y = row*area_hight + (row_end_point-row*area_hight)/2
                midpoint_x = column*area_width + (column_end_point-column*area_width)/2
                unexplored_nr = 0
                nr_cells = 0
                for row_in_area in range(row*area_hight,row_end_point+1):
                    for column_in_area in range(column*area_width,column_end_point+1):
                        nr_cells +=1
                        if self.explorer_grid[row_in_area,column_in_area] < 0.1 and self.explorer_grid[row_in_area,column_in_area] > - 0.1:
                            unexplored_nr += 1

                new_area = Area(column*area_width, row*area_hight, column_end_point, row_end_point, midpoint_x, midpoint_y, unexplored_nr, nr_cells)
                self.areas.append(new_area)
        area_grid = self.explorer_grid.copy()
        for area in self.areas:
            area_grid[area.y_start,:] = 80
            area_grid[:,area.x_start] = 80
            area_grid[area.y_end,:] = 20
            area_grid[:,area.x_end] = 20
        self.publish_grid_map(area_grid, "area")
        self.current_exploring_area_index = 0
    
    # Selects the area nearest to the pose of the robot
    def select_next_area(self):
        current_x, current_y, ang = self.retrieve_robot_position()
        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)
        self.get_logger().info('selecting the area nearest to the pose of the robot')
        best_dist = 100000000
        best_area = None
        for area in self.areas:
            if area.explored_once == False:
                dist_to_start = np.sqrt((current_x-area.midpoint_x)**2+ (current_y-area.midpoint_y)**2)
                if dist_to_start < best_dist:
                    best_area = area
                    best_dist = dist_to_start
        return best_area

    # Called when the user wants to explore more
    def explore_next(self):
        self.get_logger().info('continues exploring')
        unexplored_nr = 0
        if self.current_exploring_area_index == len(self.areas):
            self.get_logger().info('all areas explored')
            full_turn = False
            return "All explored", None, None, None, full_turn
        else:
            area = self.current_exploring_area

        for row in range(area.y_start,area.y_end+1):
            for column in range(area.x_start,area.x_end+1):
                if self.explorer_grid[row,column] < 0.1 and self.explorer_grid[row,column] > - 0.1:
                    unexplored_nr += 1

        if unexplored_nr < area.start_nr_unexplored*(1-self.accepted_exploration_ratio_per_area) or unexplored_nr == 0:
            area.explored_once = True
            self.current_exploring_area_index +=1
            self.get_logger().info('new area explored')
            self.current_exploring_area = self.select_next_area()
            self.look_at_points_dist_threshold = self.look_at_points_dist_threshold_new_area
            self.attempt_detect_from_current_position_threshold = self.attempt_detect_from_current_position_threshold_new_area
            self.look_from_points_yaw_threshold = self.look_from_points_yaw_threshold_new_area
            full_turn = False
            return "Area explored" , None, None, None, full_turn

        if unexplored_nr > area.tot_nr_cells*self.full_rotation_limit:
            full_turn = True
            start_time_testing_full_rotation = self.get_clock().now()
            result, x, y = self.generate_next_point_less_explored(area,unexplored_nr)
            time_testing_full_rotation = self.get_clock().now() - start_time_testing_full_rotation
            self.get_logger().info(f'full rotation tested, result: {result}, time used: {time_testing_full_rotation.nanoseconds/1e9}')
            #input("full rotation tested, push key to continue")
            yaw = None
            if result == "Fail":
                full_turn = False
                result, x, y, yaw = self.generate_next_point(area,unexplored_nr)
        else:
            full_turn = False
            result, x, y, yaw = self.generate_next_point(area,unexplored_nr)
        self.look_at_points_dist_threshold = self.look_at_points_dist_threshold_start
        self.attempt_detect_from_current_position_threshold = self.attempt_detect_from_current_position_threshold_start
        self.look_from_points_yaw_threshold = self.increment_threshold_yaw_look_from_point
        area.one_point = True
        if result == "Fail":
            area.explored_once = True
            self.current_exploring_area_index +=1
            self.get_logger().info('could not access all locations')
            full_turn = False
            return "Area explored" , None, None, None, full_turn

        return "Not done", x, y, yaw, full_turn
    
    #Generates points until a sufficiently good one is found
    def generate_next_point(self,area,unexplored_nr):
        accepted_point = False
        self.accepted_exploration_ratio = self.start_accepted_exploration_ratio*area.start_nr_unexplored/unexplored_nr

        self.accepted_exploration_ratio = np.min([1.0,self.accepted_exploration_ratio])
                
        # if area.nr_of_explores > 2:
        #     self.accepted_exploration_ratio*=0.6
        current_x, current_y, current_yaw = self.retrieve_robot_position()

        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)

        self.tot_rejected_points = 0
        self.look_at_points_nr_dist_rejected = 0
        self.tot_look_at_points_nr_dist_rejected = 0
        self.look_at_points_nr_yaw_rejected = 0
        self.look_at_points_yaw_threshold = self.look_at_points_yaw_threshold_start

        self.time_look_at_point_dist_rejected = 0
        self.time_look_at_point_yaw_rejected = 0
        self.time_look_at_point_rejected_other_reason = 0
        self.time_attempt_detect_from_current_position = 0
        self.time_look_from_point_dist_rejected = 0
        self.time_look_from_point_yaw_rejected = 0
        self.time_to_high_claim = 0

        self.look_from_points_nr_dist_rejected = 0
        self.tot_look_from_points_nr_dist_rejected = 0
        self.look_from_points_nr_yaw_rejected = 0
        self.tot_look_from_points_nr_yaw_rejected = 0
        self.look_from_points_extra_dist_threshold = self.increment_threshold_dist_look_from_point

        self.attempt_detect_from_current_position = 0


        self.get_logger().info(f"generating next point to explore area with midpoint: x = {area.midpoint_x}, y = {area.midpoint_y}, accepted exploration ratio for first point = {self.accepted_exploration_ratio}, accepted distance for first point = {self.look_at_points_dist_threshold}, accepted yaw change for first point = {self.look_at_points_yaw_threshold}, max detection range grid coordinates = {self.max_detection_range_grid_coordinates}")

        nrTry = 0
        self.nr_to_high_claim_rejected = 0
        self.tot_nr_to_high_claim_rejected = 0

        area_grid = self.explorer_grid[area.y_start:area.y_end+1, area.x_start:area.x_end+1]
        unexplored_index = np.where((area_grid!=1) & (area_grid!=-1))
        choice_index = np.random.randint(0,len(unexplored_index[0]))
        y = unexplored_index[0][choice_index]+area.y_start
        x = unexplored_index[1][choice_index]+area.x_start
        dist = np.sqrt((y-current_y)**2 + (x-current_x)**2)
        if dist >= self.approximate_area_size*2:
            self.look_at_points_dist_threshold = 1000000000
            self.attempt_detect_from_current_position_threshold = 0
            self.look_at_points_yaw_threshold = 1000000000
            self.look_from_points_dist_threshold = 1000000000
        
        self.start_time_generate_point = self.get_clock().now()
        self.tot_time = self.start_time_generate_point
        while accepted_point == False:
            nrTry += 1
            choice_index = np.random.randint(0,len(unexplored_index[0]))
            y = unexplored_index[0][choice_index]+area.y_start
            x = unexplored_index[1][choice_index]+area.x_start
            # self.explorer_grid[y,x] = 50
            # self.publish_grid_map(self.explorer_grid,"exp")
            # self.explorer_grid[y,x] = 0
            dist = np.sqrt((y-current_y)**2 + (x-current_x)**2)
            yaw_needed = np.arctan2(y-current_y,x-current_x)
            yaw_change = np.abs(yaw_needed - current_yaw)
            accepted_point, fail_reason, accepted_x, accepted_y, accepted_yaw = self.check_point_profit(x,y,area,unexplored_nr, dist,current_yaw, yaw_change, current_x, current_y)

            if fail_reason == "to high claim":
                self.get_logger().info(f"nr of to high claim: {self.tot_nr_to_high_claim_rejected}")
                self.get_logger().info(f"accepted exploration ratio: {self.accepted_exploration_ratio}")
                if self.accepted_exploration_ratio < 0.005:
                    self.get_logger().info(f"fail to explore more in area with midpoint: x = {area.midpoint_x}, y = {area.midpoint_y}")
                    return "Fail", None, None, None
        area.nr_of_explores +=1
        if self.tot_rejected_points != 0:
            self.time_look_at_point_dist_rejected = self.time_look_at_point_rejected_other_reason*(self.look_at_points_nr_dist_rejected/self.tot_rejected_points)
            self.time_look_at_point_yaw_rejected = self.time_look_at_point_rejected_other_reason*(self.look_at_points_nr_yaw_rejected/self.tot_rejected_points)
        self.tot_time =  self.get_clock().now() - self.tot_time
        time_diff = self.tot_time.nanoseconds - (self.time_to_high_claim+self.time_look_from_point_dist_rejected+self.time_look_from_point_yaw_rejected+self.time_look_at_point_rejected_other_reason+self.time_attempt_detect_from_current_position+(self.get_clock().now().nanoseconds- self.start_time_generate_point.nanoseconds))
        self.get_logger().info(f"tot time: {self.tot_time.nanoseconds /1e9}")
        self.get_logger().info(f"time diff: {time_diff/1e9}")
        self.get_logger().info(f"Time: time_to_high_claim = {self.time_to_high_claim/1e9}, time_look_from_point_dist_rejected = {self.time_look_from_point_dist_rejected/1e9},time_look_from_point_yaw_rejected = {self.time_look_from_point_yaw_rejected/1e9} time_look_at_point_yaw_rejected = {self.time_look_at_point_yaw_rejected/1e9}, time_look_at_point_rejected_total= {self.time_look_at_point_rejected_other_reason/1e9}, time_look_at_point_dist_rejected = {self.time_look_at_point_dist_rejected/1e9}, time_attempt_detect_from_current_position = {self.time_attempt_detect_from_current_position/1e9}")
        self.get_logger().info(f"nr of to high claim: {self.tot_nr_to_high_claim_rejected}")
        self.get_logger().info(f"point to explore from: x = {accepted_x}, y = {accepted_y}, yaw = {accepted_yaw}, accepted exploration ratio = {self.accepted_exploration_ratio}, accepted dist to look at = {self.look_at_points_dist_threshold}, accepted yaw change = {self.look_at_points_yaw_threshold}, accepted dist to look from point = {self.look_from_points_dist_threshold}")
        self.get_logger().info(f"rejected reasons: look-at point too far away {self.look_at_points_nr_dist_rejected} times, look-at point too large angular difference {self.look_at_points_nr_yaw_rejected} times, look-from point too far away {self.tot_look_from_points_nr_dist_rejected} times,look-from point too high yaw change {self.tot_look_from_points_nr_yaw_rejected} times")
        return "succes", accepted_x, accepted_y, accepted_yaw
            
    
    # Creates a circle around the generated point from which the point can be seen,
    # iterates through all points on the circle until it finds a sufficiently good one,
    # then maximizes discovery from this point by rotating and filling in all grids
    # within a radius around the robot
    def check_point_profit(self,x,y,area,unexplored_nr,dist, current_yaw, yaw_change, current_x, current_y):

        if self.explorer_grid[y,x] == -1 or (self.explorer_grid[y,x] == 1 and self.accepted_exploration_ratio>0.05 or dist > self.look_at_points_dist_threshold or yaw_change > self.look_at_points_yaw_threshold):
            self.tot_rejected_points += 1
            # ensures that the generated point is not too far from the robot's current position
            if dist > self.look_at_points_dist_threshold:
                self.tot_look_at_points_nr_dist_rejected += 1
                self.look_at_points_nr_dist_rejected += 1
                if self.look_at_points_nr_dist_rejected >= self.look_at_points_nr_dist_rejected_threshold:
                    self.attempt_detect_from_current_position_threshold = 0
                    self.look_at_points_dist_threshold += self.increment_threshold_dist_look_at_point
                    self.look_at_points_nr_dist_rejected = 0
            # ensures that the generated point does not require too large a yaw adjustment to be visible
            if yaw_change > self.look_at_points_yaw_threshold:
                self.look_at_points_nr_yaw_rejected +=1
                self.look_at_points_yaw_threshold += self.increment_threshold_yaw_look_at_point

            return False, "invalid generated point", None, None, None
        # adding time
        time_accepting_generated_point = self.get_clock().now() - self.start_time_generate_point
        self.time_look_at_point_rejected_other_reason = self.time_look_at_point_rejected_other_reason + time_accepting_generated_point.nanoseconds
        self.start_time_generate_point = self.get_clock().now()

        detection_rate_limiting_factor = False
        points_test_explore_from = []
        if dist <= self.max_detection_range_grid_coordinates:
            points_test_explore_from = [[current_x, current_y]]

        if self.attempt_detect_from_current_position >= self.attempt_detect_from_current_position_threshold:
            detection_rate_limiting_factor = True
            points_on_circle = self.circle_creator.circle_filler_angle_dependent(x,y,area,self.path_planner_grid.copy(),self.optimal_detection_distance, False, True, False,False,None,None,True)
            if len(points_on_circle) > self.nr_of_test_look_from_points:
                points_test_explore_from.extend(random.sample(points_on_circle, self.nr_of_test_look_from_points))
            else:
                points_test_explore_from.extend(points_on_circle)
        else:
            self.attempt_detect_from_current_position += 1
            self.look_at_points_yaw_threshold = (self.camera_angle/2)*(self.look_at_points_yaw_threshold//(self.camera_angle/2))
            self.look_at_points_yaw_threshold += (np.pi-(self.camera_angle/2)) /self.attempt_detect_from_current_position_threshold
            if self.attempt_detect_from_current_position >= self.attempt_detect_from_current_position_threshold:
                end_detect_cur_pos_time = self.get_clock().now() - self.tot_time
                print(f"\nend_detect_cur_pos_time: {end_detect_cur_pos_time.nanoseconds/1e9}\n")
                self.look_at_points_dist_threshold = 100000
                self.look_at_points_yaw_threshold = 10000#self.camera_angle/2
                self.increment_threshold_yaw_look_at_point*=10
                self.accepted_exploration_ratio = self.start_accepted_exploration_ratio*area.start_nr_unexplored/unexplored_nr
                self.accepted_exploration_ratio = np.min([1.0,self.accepted_exploration_ratio])
                self.nr_to_high_claim_rejected = 0

        self.look_from_points_dist_threshold = np.max([0,dist - self.max_detection_range_grid_coordinates]) + self.look_from_points_extra_dist_threshold

        for point_on_circle_index in range(len(points_test_explore_from)):
            dist_robot_point_on_circle = np.sqrt((points_test_explore_from[point_on_circle_index][1]-current_y)**2 + (points_test_explore_from[point_on_circle_index][0]-current_x)**2)
            yaw_change = 0
            if point_on_circle_index > 1:
                yaw_needed = np.arctan2(points_test_explore_from[point_on_circle_index][1]-current_y,points_test_explore_from[point_on_circle_index][0]-current_x)
                yaw_change = np.abs(yaw_needed - current_yaw)
            if dist_robot_point_on_circle <= self.look_from_points_dist_threshold and yaw_change <= self.look_from_points_yaw_threshold:
                yaw = np.arctan2(y-points_test_explore_from[point_on_circle_index][1],x-points_test_explore_from[point_on_circle_index][0])
                v1 = self.camera_angle/2 + yaw
                v2 = -self.camera_angle/2 + yaw
                detection_edge = self.circle_creator.circle_filler_angle_dependent(points_test_explore_from[point_on_circle_index][0],points_test_explore_from[point_on_circle_index][1],area,self.explorer_grid,self.max_detection_range, False, False, True,True,v1,v2,True)

                if len(detection_edge) >= unexplored_nr*self.accepted_exploration_ratio:

                    return True, None, points_test_explore_from[point_on_circle_index][0], points_test_explore_from[point_on_circle_index][1], yaw
                else:
                    self.nr_to_high_claim_rejected += 1
                    self.tot_nr_to_high_claim_rejected += 1
                    self.look_from_points_yaw_threshold += self.increment_threshold_yaw_look_from_point
                    self.look_from_points_extra_dist_threshold += self.increment_threshold_dist_look_from_point
                    if self.nr_to_high_claim_rejected > self.nr_to_high_claim_threshold:
                        
                        self.nr_to_high_claim_rejected = 0
                        self.accepted_exploration_ratio *= self.reduction_accepted_exploration_ratio
                    # adding time
                    time_to_high_claim =  self.get_clock().now() - self.start_time_generate_point
                    self.time_to_high_claim = self.time_to_high_claim + time_to_high_claim.nanoseconds
                    self.start_time_generate_point = self.get_clock().now()

            else:
                detection_rate_limiting_factor = False


                # adding time
                time_look_from_point_rejected = self.get_clock().now() - self.start_time_generate_point
                if self.attempt_detect_from_current_position >= self.attempt_detect_from_current_position_threshold:
                    if dist_robot_point_on_circle > self.look_from_points_dist_threshold:
                        self.look_from_points_nr_dist_rejected += 1
                        self.tot_look_from_points_nr_dist_rejected += 1
                        self.time_look_from_point_dist_rejected = self.time_look_from_point_dist_rejected + time_look_from_point_rejected.nanoseconds
                    if yaw_change > self.look_from_points_yaw_threshold:
                        self.look_from_points_nr_yaw_rejected += 1
                        self.tot_look_from_points_nr_yaw_rejected += 1
                        self.time_look_from_point_yaw_rejected = self.time_look_from_point_yaw_rejected + time_look_from_point_rejected.nanoseconds
                else:
                    self.time_attempt_detect_from_current_position = self.time_attempt_detect_from_current_position + time_look_from_point_rejected.nanoseconds
                self.start_time_generate_point = self.get_clock().now()

                if self.look_from_points_nr_dist_rejected > self.look_from_points_nr_dist_rejected_threshold:
                    self.look_from_points_extra_dist_threshold += self.increment_threshold_dist_look_from_point
                    self.look_from_points_nr_dist_rejected = 0
                if self.look_from_points_nr_yaw_rejected > self.look_from_points_nr_yaw_rejected_threshold:
                    self.look_from_points_yaw_threshold += self.increment_threshold_yaw_look_from_point
                    self.look_from_points_nr_yaw_rejected = 0

        if detection_rate_limiting_factor == True: 
            return False, "to high claim", None, None, None
        else:
            return False, "distance requirement look-from point too strict", None, None, None

    def generate_next_point_less_explored(self,area,unexplored_nr):
        accepted_point = False
        accepted_exploration_ratio = 1
        nrTry = 0
        nr_to_high_claim = 0
        area_grid = self.path_planner_grid[area.y_start:area.y_end+1, area.x_start:area.x_end+1]
        unexplored_index = np.where(area_grid!=-1)
        test_points = []
        if self.path_planner_grid[int(area.midpoint_y),int(area.midpoint_x)] != -1 :
            test_points = [[int(area.midpoint_x), int(area.midpoint_y)]]
        for i in range(self.nr_tries_full_rotation):
            choice_index = np.random.randint(0,len(unexplored_index[0]))
            y = unexplored_index[0][choice_index]+area.y_start
            x = unexplored_index[1][choice_index]+area.x_start
            test_points.append([x,y])
        if self.path_planner_grid[int(area.midpoint_y),int(area.midpoint_x)] != -1 :
            test_points.append([int(area.midpoint_x), int(area.midpoint_y)])

        any_point_accepted, accepted_x, accepted_y = self.check_point_profit_less_explored(test_points,area,unexplored_nr)

        if any_point_accepted:
            return "succes", accepted_x, accepted_y
        
        return "Fail", None, None
        

    def check_point_profit_less_explored(self, test_points,area,unexplored_nr):
        accepted_exploration_ratio = self.accepted_exploration_ratio_per_area
        for point_index in range(len(test_points)):

            detection_edge = self.circle_creator.circle_filler_angle_dependent(test_points[point_index][0],test_points[point_index][1],area,self.explorer_grid,self.max_detection_range, False, False, True,True,None,None,True)
            explored_cells = len(detection_edge)
            #print(f'Point, x: {test_points[point_index][0]}, y: {test_points[point_index][1]}, ration: {explored_cells/unexplored_nr}')
            if explored_cells >= unexplored_nr*accepted_exploration_ratio:
                any_point_accepted = True
                #print(accepted_exploration_ratio)
                return any_point_accepted, test_points[point_index][0], test_points[point_index][1]
            if point_index >= self.nr_tries_full_area:
                
                accepted_exploration_ratio*=self.full_rotation_reduction_accepted_exploration_ratio
        #print(accepted_exploration_ratio)
        any_point_accepted = False
        return any_point_accepted, None, None

    def publish_grid_map(self, grid, type):

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
        
        if type == "obs":
            self.path_grid_pub.publish(msg)
            self.get_logger().info("path grid published")
        if type == "exp":
            self.explore_grid_pub.publish(msg)
            self.get_logger().info("explorer grid published")
        if type == "area":
            self.area_grid_pub.publish(msg)
            self.get_logger().info("area grid published")

    def retrieve_robot_position(self):
        # rclpy.spin_once(self, timeout_sec=.5)
        self.executor.spin_once(timeout_sec=.5)
        time = rclpy.time.Time(seconds=0)

        try:
            t = self.tf_buffer.lookup_transform(
            "map",
            "base_link",
            time,
            timeout=rclpy.duration.Duration(seconds=2)
            )
            self.get_logger().info(f'Retrieved transform: x={t.transform.translation.x}, y={t.transform.translation.y}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"map"} to {"base_link"}: {ex}')
            return
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2))
        return t.transform.translation.x*100, t.transform.translation.y*100, yaw
    
    def transforme_to_map(self, goal_pose):
        # rclpy.spin_once(self, timeout_sec=.01)
        self.executor.spin_once(timeout_sec=.01)
        time = rclpy.time.Time(seconds=0)

        try:
            t = self.tf_buffer.lookup_transform(
            "map",
            "base_link",
            time,
            timeout=rclpy.duration.Duration(seconds=2)
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"map"} to {"base_link"}: {ex}')
            return
        transformed_pose= tf2_geometry_msgs.do_transform_pose(goal_pose, t)
        return transformed_pose
        
    # x, y in cm
    def convert_to_grid_coordinates(self, x, y):
        return int(np.round((x/self.resolution)+self.origin[0],0)), int(np.round((y/self.resolution)+self.origin[1],0))

    def convert_to_real_world_coordinates(self, x, y):
        return (x - self.origin[0])*self.resolution, (y - self.origin[1])*self.resolution
    
    def full_turn(self):
        ticks_per_revolution = 6

        start_time = self.get_clock().now()
        self.send_camera_request()
        print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")

        for tick in range(ticks_per_revolution - 1):
            request_msg = MoveTo.Request()
            goal_list = Path()
            goal_list.header.frame_id = "map"
            goal_list.header.stamp = self.get_clock().now().to_msg()
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 0.0
            qw = np.cos(np.pi/ticks_per_revolution) 
            qz = np.sin(np.pi/ticks_per_revolution)
            goal_msg.pose.orientation.z = float(qz)
            goal_msg.pose.orientation.w = float(qw)
            goal_msg.pose = self.transforme_to_map(goal_msg.pose)
            goal_list.poses.append(goal_msg)
            request_msg.path = goal_list
            request_msg.max_speed = 0.3
            request_msg.max_turn_speed = 0.2
            request_msg.enforce_orientation = True
            request_msg.stop_at_goal = True
            self.get_logger().info(f'Giving the camra time to detect objects, tick nr: {tick}')
            self.send_move_goal(request_msg)
            start_time = self.get_clock().now()
            self.send_camera_request()
            #print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")
            while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=0.1):
                # rclpy.spin_once(self, timeout_sec=0.01)
                self.executor.spin_once(timeout_sec=0.01)
        current_x, current_y, yaw = self.retrieve_robot_position()

        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)
        detection_edge = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.explorer_grid,self.max_detection_range, True, False, False,True,None,None,True)
        detection_edge = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.safe_space_grid,self.max_detection_range, True, False, False,True,None,None,True)
        self.publish_grid_map(self.explorer_grid,"exp")

    def part_turn(self, yaw):

        start_time = self.get_clock().now()
        self.send_camera_request()
        print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")

        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose = self.transforme_to_map(goal_msg.pose)

        qw = np.cos(yaw/2) 
        qz = np.sin(yaw/2)
        goal_msg.pose.orientation.z = float(qz)
        goal_msg.pose.orientation.w = float(qw)
        goal_list.poses.append(goal_msg)
        request_msg.path = goal_list
        request_msg.max_speed = 0.3
        request_msg.max_turn_speed = 0.2
        request_msg.enforce_orientation = True
        request_msg.stop_at_goal = True
        self.get_logger().info(f'goal orientation in map coordinates: qz = {goal_msg.pose.orientation.z}, qw = {goal_msg.pose.orientation.w}')
        self.get_logger().info(f'Giving the camra time to detect objects')
        self.send_move_goal(request_msg)
        start_time = self.get_clock().now()
        self.send_camera_request()
        print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=0.1):
            # rclpy.spin_once(self, timeout_sec=0.01)
            self.executor.spin_once(timeout_sec=0.01)

        current_x, current_y, yaw = self.retrieve_robot_position()
        v1 = self.camera_angle/2 + yaw
        v2 = -self.camera_angle/2 + yaw
        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)
        detection_edge = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.explorer_grid,self.max_detection_range, True, False, False,True, v1, v2,True)
        detection_edge = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.safe_space_grid,self.max_detection_range, True, False, False,True, v1, v2,True)
        self.publish_grid_map(self.explorer_grid,"exp")

    def is_safe_to_move(self):
        self.send_camera_request()
        current_x, current_y, yaw = self.retrieve_robot_position()
        v1 = self.camera_angle/2 + yaw
        v2 = -self.camera_angle/2 + yaw
        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)
        detection_edge = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.explorer_grid,self.max_detection_range, True, False, False,True,v1,v2,True)
        detection_edge = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.safe_space_grid,self.max_detection_range, True, False, False,True,v1,v2,True)
        self.publish_grid_map(self.explorer_grid,"exp")

    def send_camera_request(self):
        request = YoloImageDetect.Request()
        request.camera_name = "rgbd_camera"
        request.target_frame = "map"


        future = self.camera_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        while not future.done():
            self.executor.spin_once(timeout_sec=0.1) 

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Received response: {len(response.objects)} objects detected.")

            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                category = obj.category
                print(f"Object {i + 1}:")
                print(f"  Category: {category}")
                x_center = (pose.pose.position.x)
                y_center = (pose.pose.position.y)
                z_center = (pose.pose.position.z)
                print(f"  Position: {x_center} m, {y_center} m, {z_center} m")

                pose = obj.topleft_point
                x_tl = (pose.pose.position.x)
                y_tl = (pose.pose.position.y)
                z_tl = (pose.pose.position.z)
                print(f"  topleft_point: {x_tl} m, {y_tl} m, {z_tl} m")

                pose = obj.bottomright_point
                x_br = (pose.pose.position.x)
                y_br = (pose.pose.position.y)
                z_br = (pose.pose.position.z)
                print(f"  bottomright_point: {x_br} m, {y_br} m, {z_br} m")
                if category != "no_detection":
                    self.insert_object_in_map_file(x_center, y_center, category)
                
        else:
            self.get_logger().error('Failed to receive response from camera.')

    def publish_detected_objects(self): 
        for i, object in enumerate(self.detected_objects): 
            x_center, y_center, category = object
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects_or_boxes"
            marker.id = i
            marker.pose = PoseStamped().pose
            point = Point()
            point.x = x_center
            point.y = y_center
            point.z = 0.0
            marker.points.append(point)
            marker.pose.position.x = x_center
            marker.pose.position.y = y_center
            marker.pose.position.z = 0.025
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
                marker.pose.position.z = 0.05
            
            self.object_pub.publish(marker)
        
    def insert_object_in_map_file(self, x_center, y_center, category):
        self.detected_objects.append([x_center, y_center, category])
        self.publish_detected_objects()

        self.get_logger().info(f'insert object in map file')
        max_x, max_y = self.convert_to_grid_coordinates((x_center)*100+self.padding+7, (y_center)*100+self.padding+7)
        min_x, min_y = self.convert_to_grid_coordinates((x_center)*100-self.padding-7, (y_center)*100-self.padding-7)
        self.get_logger().info(f'x cells covered by object: {min_x} to {max_x}')
        self.get_logger().info(f'y cells covered by object: {min_y} to {max_y}')
        hight = self.explorer_grid.shape[0]
        width = self.explorer_grid.shape[1]
        if min_x < width and max_x > 0 and min_y > 0 and max_y < hight:
            for x in range(min_x, max_x):
                for y in range(min_y, max_y):
                    if x>0 and x<width and y>0 and y<hight:
                        self.path_planner_grid[y,x] = -1
        obs_grid = self.path_planner_grid.copy()
        self.publish_grid_map(obs_grid, "obs")

    def move_out_from_occupied(self):
        self.get_logger().info('try to move out from occupied')
        unoccupied_found = False
        current_x, current_y, current_yaw = self.retrieve_robot_position()
        current_x, current_y = self.convert_to_grid_coordinates(current_x, current_y)
        test_radius = 2*self.resolution
        while not(unoccupied_found):
            test_points = self.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,self.path_planner_grid.copy(),test_radius, False, True, False,False,None,None,True)
            for point in test_points:
                if self.path_planner_grid[point[1],point[0]] != -1:
                    safe_point = (point[0], point[1])
                    unoccupied_found = True
                    self.move([safe_point],True)
                    return
                    
            test_radius += self.resolution

    def send_move_goal(self, goal_msg):
        self.get_logger().info('sending move goal')

        future = self.move_client.call_async(goal_msg)
        # executor = rclpy.executors.SingleThreadedExecutor()
        # executor.add_node(self)
        # rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info('Waiting for move goal to be done')
        while not future.done():
            self.executor.spin_once(timeout_sec=0.1) 

        self.get_logger().info('Giving the service time to finish')
        start_time = self.get_clock().now()
        
        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=.05):
            self.executor.spin_once(timeout_sec=0.01)
            # rclpy.spin_once(self, timeout_sec=0.01)

        print("Väntat")
        move_result = future.result()
        print("klar")
        

    def move(self, waypoints, made_it_all_the_way):
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        for point in waypoints:
            x, y = point
            x, y = self.convert_to_real_world_coordinates(x,y)
            print("Waypoint koordinates real world coordinates")
            print(x)
            print(y)
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = x/100
            goal_msg.pose.position.y = y/100
            goal_msg.pose.orientation.z = 0.0 
            goal_msg.pose.orientation.w = 1.0
            goal_list.poses.append(goal_msg)
        request_msg.path = goal_list
        request_msg.max_speed = 0.4
        request_msg.max_turn_speed = 0.25
        request_msg.enforce_orientation = False
        request_msg.stop_at_goal = True
        request_msg.allow_reverse = True
        if made_it_all_the_way == False:
            request_msg.allow_reverse = False
        self.get_logger().info(f'sending a move command with {len(request_msg.path.poses)} waypoints')
        self.send_move_goal(request_msg)
        

    def move_to_new_explore_point(self, obs_grid, goal_x, goal_y, goal_yaw, full_turn):
        start_x, start_y, ang = self.retrieve_robot_position()
        start_x, start_y = self.convert_to_grid_coordinates(start_x, start_y)
        self.get_logger().info(f'current position in grid coordinates: x = {start_x} y = {start_y}')
        self.get_logger().info(f'goal position in grid coordinates: x = {goal_x} y = {goal_y}')

        grid1, path_result = self.planner.A_star(obs_grid, [start_x, start_y], [goal_x,goal_y], 1, False, 1, True)
        #input(f"innan test move_out_from_occupied, path result: {path_result}")
        if path_result == False:
            self.move_out_from_occupied()
            return
        #input("Efter test move_out_from_occupied")
        safe_space_grid_with_obstacles = self.safe_space_grid.copy()
        safe_space_grid_with_obstacles[grid1 == -1] = -1
        waypoints, made_it_all_the_way = self.planner.waypoint_creator(True, safe_space_grid_with_obstacles)
        for point in waypoints:
            x,y = point
            grid1[y,x] = 90
        self.get_logger().info(f'unsafe waypoints: {waypoints}')

        grid1[goal_y,goal_x] = 50
        grid1[start_y,start_x] = 25
        
        self.publish_grid_map(grid1, "obs")
        #input("Push a key to move along path")
        self.move(waypoints, made_it_all_the_way)


        if made_it_all_the_way == True:
            if full_turn:
                self.full_turn()
            else:
                self.part_turn(goal_yaw)
        else:
            self.is_safe_to_move()
    
    def run_explorer(self):
        self.explorer_grid = self.fetch_map(0)
        self.divide_grid_areas()
        self.path_planner_grid = self.fetch_map(self.padding)
        self.safe_space_grid = self.path_planner_grid.copy()
        self.current_exploring_area_index = 0
        self.current_exploring_area = self.select_next_area()
        self.planner = path_planner()
        self.circle_creator = circle_creator(self.resolution)
        self.full_turn()


        while self.status != "All explored":
            self.status = ""
            x = []
            y = []

            if self.status != "Area explored" and self.status != "All explored":
                self.status, new_x, new_y, new_yaw, full_turn = self.explore_next()
                if self.status == "Not done":
                    x.append(new_x)
                    y.append(new_y)
                    #input("Push a key to generate path")
                    obs_grid = self.path_planner_grid.copy()
                    self.move_to_new_explore_point(obs_grid, new_x,new_y, new_yaw, full_turn)

            print(self.status)
                
            #input("Push a key to continue exploring")

    
#_start and _end are also included in the area
class Area:
    def __init__(self,x_start,y_start,x_end,y_end,midpoint_x,midpoint_y,start_nr_unexplored,nr_cells):
        self.x_start = x_start
        self.y_start = y_start
        self.x_end = x_end
        self.y_end = y_end
        self.explored_once = False
        self.one_point = False
        self.midpoint_x = midpoint_x
        self.midpoint_y = midpoint_y
        self.nr_of_explores = 0
        self.start_nr_unexplored = start_nr_unexplored
        self.tot_nr_cells = nr_cells



def main():
    rclpy.init()
    node = Explorer_node()
    
    # creating executor
    node.circle_creator = None
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    node.executor = ex

    node.run_explorer()
    try:
        node.ex.spin()
        # rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
