import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from core_interfaces.srv import GridCreator
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from core_interfaces.srv import MoveToObject, MoveTo

from navigation.path_planner import path_planner

from core_interfaces.srv import MoveTo
from nav_msgs.msg import Path

from math import cos, sin, atan2, fabs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from navigation.circle_creator_uppdate import circle_creator
import os
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time as std_time
from _thread import start_new_thread
import time

class PickUp(Node):
    def __init__(self):
        super().__init__('PickUp')
        self.get_logger().info('Pick up node started 1')

        self.collection_workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_3.tsv"
        self.objects_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/map_3.tsv"

        self.resolution = 5 #cm / cell
        self.padding = 25 #cm
        self.radius = 50 #cm
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

        self.start_obs_grid = None

        group = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()

        self.grid_fill_cli = self.create_client(GridCreator,"fill_in_workspace")

        self.pickup_grid_pub = self.create_publisher(OccupancyGrid, '/pick_up_grid_map', 10)
        
        self.current_pickup_pose_pub = self.create_publisher(PoseStamped, '/current_pickup_pose', 10) 

        while not self.grid_fill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('grid filler service not available, waiting again...')

        self.move_client = self.create_client(MoveTo,"MoveTo")
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move service not available, waiting again...')
        
        self.pickup_service = self.create_service(MoveToObject, "move_to_object", self.move_to_object_callback, callback_group=group)
        
        # Creating buffer and transform listener
        buffer_size = rclpy.duration.Duration(seconds=10.0) 
        self.tf_buffer = Buffer(cache_time=buffer_size)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        
        self.executor = None

    def set_executor(self, executor):
        self.executor = executor

    # callback, moves to the desired object
    def move_to_object_callback(self, request, response):
        if request.target == "Go BACK":
            self.get_logger().info(f'Objects left before pickup: {len(self.objects)}')
            if self.latest_object_index != None:
                self.objects.pop(self.latest_object_index)
                self.latest_object_index = None
            self.get_logger().info(f'Objects left after pickup: {len(self.objects)}')
            go_back_command = self.create_go_back_command()
            success = self.move_along_path(go_back_command)
            self.get_logger().info(f'Success with go back: {success}')
            response.success = success
            response.message = "Done"
            response.finished = False
            if len(self.objects) == 0:
                response.finished = True
            return response
        else:
        
            x, y = self.retrieve_object_position(request)
            self.get_logger().info(f'Object position: x = {x} y = {y}')
            x_goal, y_goal = self.retrieve_pick_up_position(x,y)
            self.get_logger().info(f'Pick up position: x = {x_goal} y = {y_goal}')
            angle_between_object_and_pickup = np.arctan2(y - y_goal, x - x_goal)
            start_x, start_y = self.retrieve_robot_position()
            self.get_logger().info(f'Robot position: x = {start_x} y = {start_y}')
            start_x, start_y = self.convert_to_grid_coordinates(start_x, start_y)
            self.get_logger().info(f'Robot position (after conversion): x = {start_x} y = {start_y}')
            obs_grid = self.start_obs_grid.copy()
            path = self.generate_path(obs_grid, start_x, start_y, x_goal, y_goal)
            self.get_logger().info(f'Path: {path}')
            move_command = self.create_move_command(path, angle_between_object_and_pickup)

            success = self.move_along_path(move_command)
            self.get_logger().info(f'Success with move: {success}')
            if not success:
                response.success = False
                response.finished = False
                response.message = "Failed to move to object"
                return response

            turn = self.generate_turn()

            self.get_logger().info(f'Turn: {turn}')
            turn_command = self.create_turn_command(turn)
            self.get_logger().info(f'Turn command: {turn_command}')
            success = self.move_along_path(turn_command)
            self.get_logger().info(f'Success with turn: {success}')
            if not success:
                response.success = False
                response.finished = False
                response.message = "Failed to turn to object"
                return response

            response.success = True
            response.finished = False
            response.message = "Done"
            return response
    
    def create_go_back_command(self):
        self.get_logger().info("create_go_back_command")
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        x, y = self.go_back_position[0], self.go_back_position[1]
        self.get_logger().info("Go back koordinates")
        self.get_logger().info(f"position x {x}, position y {y}")
        self.get_logger().info(f"orientation z {self.go_back_oriorientation_z}, orientation w {self.go_back_oriorientation_w}")
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
        return(request_msg)

    # receives the desired object, returns the position of the nearest one
    def retrieve_object_position(self,request):
        self.get_logger().info('move to object callback')
        if request.target == "BOX":
            self.distance_to_object = self.distance_to_object_during_place
            index = self.select_object(self.boxes)
            self.next_object = self.boxes[index]
        if request.target == "OBJECT":
            self.distance_to_object = self.distance_to_object_during_pickup
            index = self.select_object(self.objects)
            self.latest_object_index = index
            self.next_object = self.objects[index]
            self.get_logger().info(f'Next object to pick up: x = {self.next_object[0]} y = {self.next_object[1]}, ojects left = {len(self.objects)}')

        x = self.next_object[0]
        y = self.next_object[1]
        x, y = self.convert_to_grid_coordinates(x,y)
        return x, y
    
    # returns the index of the nearest object in the list
    def select_object(self, list_of_ojects):
        current_x, current_y = self.retrieve_robot_position()
        self.get_logger().info('selecting the object nearest to the pose of the robot')
        best_dist = 100000000
        best_object = None 
        self.get_logger().info(str(list_of_ojects))
        for object_index in range(len(list_of_ojects)):
            dist_to_start = np.sqrt((current_x-list_of_ojects[object_index][0])**2+ (current_y-list_of_ojects[object_index][1])**2)
            if dist_to_start < best_dist:
                best_object_index = object_index
                best_dist = dist_to_start
        return best_object_index
    
    # receives the position of the object to be picked up, returns the position from where the pickup should start
    def retrieve_pick_up_position(self, x, y):
        points_on_circle = self.circle_creator.circle_filler_angle_dependent(x,y,None,self.grid,self.radius,False,True, False,False,None,None,True)
        self.get_logger().info(f"points_on_circle before convert to real world coordinates:{points_on_circle}")
        for i in range(len(points_on_circle)):
            tmp_x, tmp_y = self.convert_to_real_world_coordinates(points_on_circle[i][0], points_on_circle[i][1])
            points_on_circle[i] = [tmp_x, tmp_y]
            self.get_logger().info(f"{[float(tmp_x), float(tmp_y)]=}")
        self.get_logger().info(f"points_on_circle after convert to real world coordinates:{points_on_circle}")
        position_index = self.select_object(points_on_circle)
        goal_point = points_on_circle[position_index]
        self.go_back_position = goal_point
        x_goal, y_goal = self.convert_to_grid_coordinates(goal_point[0], goal_point[1])
        return x_goal, y_goal

    # generates a path between the start and goal based on the grid map
    def generate_path(self, grid, start_x, start_y, goal_x, goal_y):
        grid1, path_result = self.planner.A_star(grid, [start_x, start_y], [goal_x,goal_y], 1, False, 1, True)
        waypoints, made_it_all_the_way = self.planner.waypoint_creator(False, self.grid)

        self.get_logger().debug(f"grid1: {grid1}")
        for point in waypoints:
            x,y = point
            grid1[y,x] = 50

        grid1[goal_y,goal_x] = 25
        grid1[start_y,start_x] = 75
        
        self.publish_grid_map(grid1)
        self.get_logger().info('grid map with path published')
        self.get_logger().info(str(len(waypoints)))
        return waypoints

    # creates commands to drive along the path
    def create_move_command(self, waypoints, angle_between_object_and_pickup):
        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        for point in waypoints:
            x, y = point
            self.get_logger().info(f"Before conversion: {x}, {y}, origin: , {self.origin}")
            x, y = self.convert_to_real_world_coordinates(x,y)
            self.get_logger().info("Waypoint koordinates (after conversion)")
            self.get_logger().info(str(x))
            self.get_logger().info(str(y))
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
        request_msg.allow_reverse = True
        request_msg.stop_at_goal = True
        return(request_msg)

     # generates a turn to face the object and a forward movement to approach it closely
    def generate_turn(self):
        goal_x, goal_y, goal_angle = self.next_object
        goal_pose_base_link = PoseStamped()
        
        goal_pose_base_link.header.frame_id = "map"
        goal_pose_base_link.header.stamp = self.get_clock().now().to_msg()
        goal_pose_base_link.pose.position.x = goal_x/100
        goal_pose_base_link.pose.position.y = goal_y/100 

        self.current_pickup_pose_pub.publish(goal_pose_base_link)
        goal_pose_base_link.pose = self.transform_to_base_link(goal_pose_base_link.pose)
        return goal_pose_base_link

    # creates commands to do the turn
    def create_turn_command(self, goal_pose_base_link):
        goal_pose_odom = PoseStamped()
        goal_pose_odom.pose = self.transform_to(goal_pose_base_link.pose, "odom", "base_link")
        current_pose_odom = PoseStamped()
        current_pose_odom.pose = self.transform_to(current_pose_odom.pose, "odom", "base_link")

        angle_between_current_pose_and_object = np.arctan2(goal_pose_odom.pose.position.y - current_pose_odom.pose.position.y, goal_pose_odom.pose.position.x - current_pose_odom.pose.position.x)

        pickup_distance = 0.16 
        arm_base_link_y_shift = 0.0475  
        base_link_to_pickup_radius = (pickup_distance**2 + arm_base_link_y_shift**2)**0.5
        base_link_pickup_angle = np.arctan2(arm_base_link_y_shift, pickup_distance) 
        target_angle = angle_between_current_pose_and_object + base_link_pickup_angle  
        # target_angle = - target_angle

        object_pos = np.array([goal_pose_base_link.pose.position.x, goal_pose_base_link.pose.position.y])
        distance_to_object = np.linalg.norm(object_pos)
        target_pos = object_pos - object_pos / distance_to_object * base_link_to_pickup_radius 

        # shift = 0.05
        # dist = np.sqrt(goal_pose_base_link.pose.position.x**2 + goal_pose_base_link.pose.position.y**2)

        # obj_orthogonal_direction = np.array([-goal_pose_base_link.pose.position.y / dist, goal_pose_base_link.pose.position.x / dist])
        # goal_pose_base_link.pose.position.x += shift * obj_orthogonal_direction[0] 
        # goal_pose_base_link.pose.position.y += shift * obj_orthogonal_direction[1] 

        # dist = np.sqrt(goal_pose_base_link.pose.position.x**2 + goal_pose_base_link.pose.position.y**2)
        # angle = np.arctan2(goal_pose_base_link.pose.position.y, goal_pose_base_link.pose.position.x)

        # forward_move = dist - self.distance_to_object / 100
        # # Add debug prints 
        # self.get_logger().info(f'goal_pose_base_link.pose.position.x = {goal_pose_base_link.pose.position.x}')
        # self.get_logger().info(f'goal_pose_base_link.pose.position.y = {goal_pose_base_link.pose.position.y}')
        # self.get_logger().info(f'angle = {angle}')
        # self.get_logger().info(f'dist = {dist}')

        request_msg = MoveTo.Request()
        goal_list = Path()
        goal_list.header.frame_id = "map"
        goal_list.header.stamp = self.get_clock().now().to_msg()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        # goal_msg.pose.position.x = forward_move * np.cos(angle)
        # goal_msg.pose.position.y = forward_move * np.sin(angle)
        # qw = np.cos(angle/2) 
        # qz = np.sin(angle/2)
        goal_msg.pose.position.x = target_pos[0]
        goal_msg.pose.position.y = target_pos[1]
        qw = np.cos(target_angle/2) 
        qz = np.sin(target_angle/2)
        self.go_back_oriorientation_z = float(qz)
        self.go_back_oriorientation_w = float(qw)
        goal_msg.pose.orientation.z = float(qz)
        goal_msg.pose.orientation.w = float(qw)
        self.get_logger().info(f'Before transform: {goal_msg.pose}')
        goal_msg.pose = self.transform_to_map(goal_msg.pose)
        self.get_logger().info(f'After transform: {goal_msg.pose}')
        goal_list.poses.append(goal_msg)
        request_msg.path = goal_list
        request_msg.max_speed = 0.2
        request_msg.max_turn_speed = 0.15
        request_msg.enforce_orientation = True
        request_msg.allow_reverse = False
        request_msg.stop_at_goal = True
        return request_msg
    
    def move_callback(self, future):
        self.done_move = True

    def turn_callback(self, future):
        self.done_turn = True
        
    # sends drive commands to move the robot
    def move_along_path(self,move_command):
        self.get_logger().info('sending move goal')

        future = self.move_client.call_async(move_command)

        self.get_logger().info('Waiting for move goal to be done')

        # self.executor.spin_until_future_complete(future)
        while not future.done():
            self.get_logger().info('Waiting for move goal to be done')
            self.executor.spin_once(timeout_sec=0.1)
        # future.add_done_callback(self.move_callback) 
        
        self.get_logger().info('Move goal done')
        result = future.result() 
        self.get_logger().info(f'-------------Move goal result: {result}')
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
            self.get_logger().info(
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
            self.get_logger().info(f'Retrieved transform: x={t.transform.translation.x}, y={t.transform.translation.y}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"map"} to {"base_link"}: {ex}')
            return
        return t.transform.translation.x*100, t.transform.translation.y*100

    
    def convert_to_grid_coordinates(self, x, y):
        return int(np.round((x/self.resolution)+self.origin[0],0)), int(np.round((y/self.resolution)+self.origin[1],0))

    def convert_to_real_world_coordinates(self, x, y):
        return (x - self.origin[0])*self.resolution, (y - self.origin[1])*self.resolution

    # places the objects in the grid map before publishing
    def insert_objects_in_map_file(self, grid):
        grid2 = grid.copy()
        for object in self.objects:
            x, y = self.convert_to_grid_coordinates(object[0],object[1])
            fill_in_cells = self.circle_creator.circle_filler_angle_dependent(x,y,None,grid2,self.padding,False,False, False,True,None,None,False)
            for cell in fill_in_cells:
                grid[cell[1],cell[0]] = -1
            grid[y,x] = 90
        half_diagonal_box = np.sqrt((24/2)**2+(15/2)**2)
        for box in self.boxes:
            x_max, y_max = self.convert_to_grid_coordinates(box[0]+half_diagonal_box+self.padding,box[1]+half_diagonal_box+self.padding)
            x_min, y_min = self.convert_to_grid_coordinates(box[0]-half_diagonal_box-self.padding,box[1]-half_diagonal_box-self.padding)
            x_max = int(np.min([int(grid.shape[1])-1, x_max]))
            y_max = int(np.min([int(grid.shape[0])-1, y_max]))
            x_min = int(np.max([0, x_min]))
            y_min = int(np.max([0, y_min]))
            for cell_x in range(x_min,x_max+1):
                for cell_y in range(y_min,y_max+1):
                    grid[cell_y, cell_x] = -1
            mid_box_x, mid_box_y = self.convert_to_grid_coordinates(box[0],box[1])
            grid[mid_box_y,mid_box_x] = 90

        self.get_logger().info(f'all objects inserted in grid')
    
    # publishes the grid map.
    def publish_grid_map(self, grid):
        self.get_logger().info(f'In start of publishing grid map')
        
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
        self.get_logger().info(f'Publishing grid map')
        self.pickup_grid_pub.publish(msg)

    # retrieve objects from file
    def retrieve_objects_from_file(self):
        self.get_logger().info('retrieve objects from file')
        file = self.objects_file
        read_start_row = 0 
        file_path = os.path.expanduser(file)
        while True:
            try:
                data = np.loadtxt(file_path, delimiter="\t", dtype = "str")
                break
            except ValueError as e:
                read_start_row += 1
        self.get_logger().info(str(data))
        for object in data:
            if object[0] == 'B':
                self.boxes.append([float(object[1]),float(object[2]), float(object[3])])
            else:
                self.objects.append([float(object[1]),float(object[2]), float(object[3])])
        self.get_logger().info('objects retrieved from file')
        self.insert_objects_in_map_file(self.grid)
        self.start_obs_grid = self.grid.copy()
        self.publish_grid_map(self.grid)

    # fetch grid map
    def fetch_map(self):
        self.get_logger().info('fetching map')
        req = GridCreator.Request()
        req.padding = self.padding
        req.resolution = self.resolution
        req.file_name = self.collection_workspace_file
        future = self.grid_fill_cli.call_async(req)
        self.executor.spin_until_future_complete(future)

        flat_grid = future.result()
        self.origin = [flat_grid.grid.info.origin.position.x, flat_grid.grid.info.origin.position.y]
        self.grid = np.array(flat_grid.grid.data,dtype=np.float64).reshape((flat_grid.grid.info.height, flat_grid.grid.info.width))
        
        self.get_logger().info('map fetched')
        self.circle_creator = circle_creator(self.resolution)


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