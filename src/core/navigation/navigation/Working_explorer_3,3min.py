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


class Explorer_node(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info('explorer node started 3')

        self.exploration_workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_2.tsv"
        self.resolution = 5 #cm / cell
        self.padding = 28 #cm
        self.max_detection_range = 100 #cm
        self.optimal_detection_distance = 50 #cm
        self.approximate_area_size = 130 #cm
        self.start_accepted_exploration_ratio = np.min([1.0,0.54*(np.pi*self.max_detection_range**2)/(self.approximate_area_size**2)])
        self.explorer_grid = None
        self.path_planner_grid = None
        self.origin = None
        self.current_exploring_area = None
        self.areas = []

        self.status = ""
        self.planner = path_planner()

        group2 = ReentrantCallbackGroup()
        group3 = ReentrantCallbackGroup()
        group4 = ReentrantCallbackGroup()

        self.grid_fill_cli = self.create_client(GridCreator,"fill_in_workspace")
        self.path_grid_pub = self.create_publisher(OccupancyGrid, '/path_grid_map', 10, callback_group=group2)
        self.explore_grid_pub = self.create_publisher(OccupancyGrid, '/explore_grid_map', 10, callback_group=group3)
        self.area_grid_pub = self.create_publisher(OccupancyGrid, '/area_grid_map', 10, callback_group=group4)


        while not self.grid_fill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('grid filler service not available, waiting again...')

        self.move_client = self.create_client(MoveTo,"MoveTo", callback_group=group4)

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move service not available, waiting again...')

        #self.camera_client = self.create_client(YoloImageDetect, 'yolo_image_detect', callback_group=group2)

        # while not self.camera_client.wait_for_service(timeout_sec=10):
        #     self.get_logger().info('Service not available, waiting again...')
        
        #creating buffer and transform listener
        buffer_size = rclpy.duration.Duration(seconds=10.0) 
        self.tf_buffer = Buffer(cache_time=buffer_size)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        std_time.sleep(1)

        self.executor = None
        self.circle_creator = None

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
        

    # Initial: divide the grid map into areas
    def divide_grid_areas(self):
        self.get_logger().info('dividing the grid map into areas')
        hight = self.explorer_grid.shape[0]*self.resolution
        width = self.explorer_grid.shape[1]*self.resolution
        hight_divider = hight//self.approximate_area_size
        width_divider = width//self.approximate_area_size
        area_hight = int(np.ceil((hight/hight_divider)/self.resolution))
        area_width = int(np.ceil((width/width_divider)/self.resolution))
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
                
                new_area = Area(column*area_width, row*area_hight, column_end_point, row_end_point, midpoint_x, midpoint_y)
                self.areas.append(new_area)
        area_grid = self.explorer_grid.copy()
        for area in self.areas:
            area_grid[area.y_start,:] = 80
            area_grid[:,area.x_start] = 80
            area_grid[area.y_end,:] = 20
            area_grid[:,area.x_end] = 20
        self.publish_grid_map(area_grid, "area")
        self.current_exploring_area = 0
    
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
        if self.current_exploring_area == len(self.areas):
            self.get_logger().info('all areas explored')
            return "All explored", None, None
        else:
            area = self.select_next_area()

        for row in range(area.y_start,area.y_end+1):
            for column in range(area.x_start,area.x_end+1):
                if self.explorer_grid[row,column] < 0.1 and self.explorer_grid[row,column] > - 0.1:
                    unexplored_nr += 1
        if unexplored_nr == 0:
            area.explored_once = True
            self.current_exploring_area +=1
            self.get_logger().info('new area explored')
            return "Area explored" , None, None
        result, x, y = self.generate_next_point(area,unexplored_nr)
        area.one_point = True
        if result == "Fail":
            area.explored_once = True
            self.current_exploring_area +=1
            self.get_logger().info('new area explored, could not access all locations')
            return "Area explored" , None, None

        return "Not done", x, y
    
    #Generates points until a sufficiently good one is found
    def generate_next_point(self,area,unexplored_nr):
        accepted_point = False
        accepted_exploration_ratio = self.start_accepted_exploration_ratio
        nrTry = 0
        nr_to_high_claim = 0
        while accepted_point == False:
            nrTry += 1
            y = np.random.randint(area.y_start, (area.y_end+1))
            x = np.random.randint(area.x_start, area.x_end+1)

            accepted_point, fail_reason, accepted_x, accepted_y = self.check_point_profit(x,y, accepted_exploration_ratio,area,unexplored_nr)
            if fail_reason == "to high claim":
                nr_to_high_claim += 1
                if nr_to_high_claim >1:
                    accepted_exploration_ratio *= 0.5
                if nr_to_high_claim > 4:
                    return "Fail", None, None
        return "succes", accepted_x, accepted_y
            
    
    # Creates a circle around the generated point from which the point can be seen,
    # iterates through all points on the circle until it finds a sufficiently good one,
    # then maximizes discovery from this point by rotating and filling in all grids
    # within a radius around the robot
    def check_point_profit(self,x,y, accepted_exploration_ratio,area,unexplored_nr):

        if self.explorer_grid[y,x] == -1 or (self.explorer_grid[y,x] == 1 and accepted_exploration_ratio>0.05):
            return False, "on obs or free", None, None
        if area.one_point == False and self.path_planner_grid[int(area.midpoint_y),int(area.midpoint_x)] != -1 :
            points_on_circle = [[int(area.midpoint_x), int(area.midpoint_y)]]
        else:
            points_on_circle = self.circle_creator.circle_filler_angle_dependent(x,y,area,self.path_planner_grid.copy(),self.optimal_detection_distance, False, True, True,False,None,None)   

        for point_on_circle_index in range(len(points_on_circle)):

            detection_edge = self.circle_creator.circle_filler_angle_dependent(points_on_circle[point_on_circle_index][0],points_on_circle[point_on_circle_index][1],area,self.explorer_grid,self.max_detection_range, False, False, True,True,None,None)

            if len(detection_edge) >= unexplored_nr*accepted_exploration_ratio:

                return True, None, points_on_circle[point_on_circle_index][0], points_on_circle[point_on_circle_index][1]

        return False, "to high claim", None, None

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
    
    def turn(self):
        ticks_per_revolution = 6

        start_time = self.get_clock().now()
        #self.send_camera_request()
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
            #self.send_camera_request()
            #print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")
            while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=0.1):
                # rclpy.spin_once(self, timeout_sec=0.01)
                self.executor.spin_once(timeout_sec=0.01)

        #self.run_explorer()
    # def send_camera_request(self):
    #     request = YoloImageDetect.Request()
    #     request.camera_name = "rgbd_camera"
    #     request.target_frame = "map"


    #     future = self.camera_client.call_async(request)
    #     # rclpy.spin_until_future_complete(self, future)
    #     while not future.done():
    #         self.executor.spin_once(timeout_sec=0.1) 

    #     if future.result() is not None:
    #         response = future.result()
    #         self.get_logger().info(f"Received response: {len(response.objects)} objects detected.")

    #         for i, obj in enumerate(response.objects):
    #             pose = obj.center_point
    #             category = obj.category
    #             print(f"Object {i + 1}:")
    #             print(f"  Category: {category}")
    #             x_center = (pose.pose.position.x)
    #             y_center = (pose.pose.position.y)
    #             z_center = (pose.pose.position.z)
    #             print(f"  Position: {x_center} m, {y_center} m, {z_center} m")

    #             pose = obj.topleft_point
    #             x_tl = (pose.pose.position.x)
    #             y_tl = (pose.pose.position.y)
    #             z_tl = (pose.pose.position.z)
    #             print(f"  topleft_point: {x_tl} m, {y_tl} m, {z_tl} m")

    #             pose = obj.bottomright_point
    #             x_br = (pose.pose.position.x)
    #             y_br = (pose.pose.position.y)
    #             z_br = (pose.pose.position.z)
    #             print(f"  bottomright_point: {x_br} m, {y_br} m, {z_br} m")
    #             if category != "no_detection":
    #                 diagonal_dist = np.sqrt((x_br-x_tl)**2+(y_br-y_tl)**2+(z_br-z_tl)**2)
    #                 self.insert_object_in_map_file(diagonal_dist, x_center, y_center)
                
    #     else:
    #         self.get_logger().error('Failed to receive response from camera.')

    # def insert_object_in_map_file(self,diagonal_dist, x_center, y_center):
    #     self.get_logger().info('insert object in map file')
    #     max_x, max_y = self.convert_to_grid_coordinates((x_center + diagonal_dist)*100+self.padding, (y_center + diagonal_dist)*100+self.padding)
    #     min_x, min_y = self.convert_to_grid_coordinates((x_center - diagonal_dist)*100-self.padding, (y_center - diagonal_dist)*100-self.padding)
    #     self.get_logger().info(f'x cells covered by object: {min_x} to {max_x}')
    #     self.get_logger().info(f'y cells covered by object: {min_y} to {max_y}')
    #     for x in range(min_x, max_x):
    #         for y in range(min_y, max_y):
    #             self.path_planner_grid[y,x] = -1

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
        

    def move(self, waypoints):
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        for point in waypoints:
            x, y = point
            x, y = self.convert_to_real_world_coordinates(x,y)
            print("Waypoint koordinates")
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
        self.get_logger().info(f'sending a move command with {len(request_msg.path.poses)} waypoints')
        self.send_move_goal(request_msg)
        

    def move_to_new_explore_point(self, obs_grid, goal_x, goal_y):
        start_x, start_y, ang = self.retrieve_robot_position()
        start_x, start_y = self.convert_to_grid_coordinates(start_x, start_y)
        self.get_logger().info(f'current position in grid coordinates: x = {start_x} y = {start_y}')
        self.get_logger().info(f'goal position in grid coordinates: x = {goal_x} y = {goal_y}')

        grid1 = self.planner.A_star(obs_grid, [start_x, start_y], [goal_x,goal_y], 1, False, 1, True)


        waypoints, made_it_all_the_way = self.planner.waypoint_creator(False, self.path_planner_grid)
        for point in waypoints:
            x,y = point
            grid1[y,x] = 90


        grid1[goal_y,goal_x] = 50
        grid1[start_y,start_x] = 25
        
        self.publish_grid_map(grid1, "obs")
        #input("Push a key to move along path")
        self.move(waypoints)
        self.turn()
    
    def run_explorer(self):

        while self.status != "All explored":
            self.status = ""
            x = []
            y = []

            if self.status != "Area explored" and self.status != "All explored":
                self.status, new_x, new_y = self.explore_next()
                if self.status == "Not done":
                    x.append(new_x)
                    y.append(new_y)
                    #input("Push a key to generate path")
                    obs_grid = self.path_planner_grid.copy()
                    self.move_to_new_explore_point(obs_grid, new_x,new_y)

            print(self.status)
            if len(x) != 0:
                # for i in range(len(x)-1):
                #     self.grid[y[i],x[i]] = 0.6
                # self.grid[y[-1],x[-1]] = -0.6
                # current_x, current_y, yaw = self.retrieve_robot_position()
                # v1 = np.pi/6 + yaw
                # v2 = -np.pi/6 + yaw
                
                detection_edge = self.circle_creator.circle_filler_angle_dependent(new_x,new_y,None,self.explorer_grid,self.max_detection_range, True, False, False,True, None, None)
                self.publish_grid_map(self.explorer_grid,"exp")
                #input("Push a key to continue exploring")

    
#_start and _end are also included in the area
class Area:
    def __init__(self,x_start,y_start,x_end,y_end,midpoint_x,midpoint_y):
        self.x_start = x_start
        self.y_start = y_start
        self.x_end = x_end
        self.y_end = y_end
        self.explored_once = False
        self.one_point = False
        self.midpoint_x = midpoint_x
        self.midpoint_y = midpoint_y



def main():
    rclpy.init()
    node = Explorer_node()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    node.executor = ex
    node.circle_creator = circle_creator(node.resolution)
    node.explorer_grid = node.fetch_map(0)
    node.divide_grid_areas()
    #input("Push a key to start exploration")
    node.path_planner_grid = node.fetch_map(node.padding)

    node.turn()
    current_x, current_y, yaw = node.retrieve_robot_position()
    v1 = np.pi/6 + yaw
    v2 = -np.pi/6 + yaw
    current_x, current_y = node.convert_to_grid_coordinates(current_x, current_y)
    detection_edge = node.circle_creator.circle_filler_angle_dependent(current_x,current_y,None,node.explorer_grid,node.max_detection_range, True, False, False,True,None,None)
    node.publish_grid_map(node.explorer_grid,"exp")

    node.run_explorer()
    try:
        ex.spin()
        # rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
