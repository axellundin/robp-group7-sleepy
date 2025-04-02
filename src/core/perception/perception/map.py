import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import time
from geometry_msgs.msg import Point, Pose
from core_interfaces.msg import YoloClassifiedObject, PointcloudDetectedObj

import cv2
import os
import math

time_it_enable = True
def time_it(func):
    """ 
    calculate the time used for executing the function
    """
    def wrapper(*args, **kwargs):
        if time_it_enable:  
            start_time = time.time() 
            result = func(*args, **kwargs) 
            end_time = time.time()  
            elapsed_time = end_time - start_time 
            print(f"Function '{func.__name__}' executed in {elapsed_time:.4f} seconds")
        else:
            result = func(*args, **kwargs)  
        return result  
    return wrapper

class MapNode(Node):
    def __init__(self, width=100, height=100, resolution=0.03):
        super().__init__('map_node')

        self.publisher = self.create_publisher(OccupancyGrid, 'grid_map', 10)

        self.width = width 
        self.height = height 
        self.resolution = resolution  
        self.origin = [-0.5*width*resolution, -0.5*height*resolution]  # unit is meter

        self.grid_map = np.zeros((self.height, self.width), dtype=np.int8)
        self.grid_map.fill(-1) # -1 unexplored, 0 explored, 100 obstacle, 99 obstacle in lidarfromcam, 21 toy 22 cube 23 ball 24 box  20 unknown

        self.cache_map = np.zeros((self.height, self.width), dtype=np.int8)
        self.cache_map.fill(-1)

        self.intention_map = np.zeros((self.height, self.width, 5), dtype=np.int8) 

        self.robot_pose = Pose()
        self.yolo_category_list = None

        self.mappoint_dictlist = [] # every row is a dict for point already on map, key index value int, key point value posestamped, key class value string
        self.intentionpoint_dictlist = [] # every row is a dict for point in intention, key index value int, key point value posestamped, key intention value list like [1,2,3,4,5]
        self.pointcloud_msg_list = [] # inside is just msg 
        self.yolo_msg_list = [] # msg inside 

        #params
        self.avoid_obj_threshold = 0.05 # if the distance between obj and obstacle is less than this threshold, we skip this obj
        self.tolerance_between_obj_and_lidarfromcam = 0.1 # if obj and lidarfromcam closer than this, then it is invisible 
        self.same_obj_existed_before = 0.1 # if obj existed before close to report this time, we think it is the same obj and existed before
        self.fading = True

        #yolo params
        self.yolopoint_mapping_height_threshold = 0.5 # obj above this height threshold from yolo detector is not resonable, as objs are on the ground 
        self.yolopoint_mapping_remove_overlap_threshold = 0.04 #if 2 reported obj is closer than this threshold, it could be fake overlap illusion and we skip one 
        self.align_yolo_to_pc_radius_threshold = 0.2# distance between pointcloud detection and yolo detection if closer than this, then the obj from yolo is the obj detected by pointcloud

        #intention params
        self.intention_value_range = 20
        self.have_obj_threshold = 12  
        self.have_class_threshold = 12 
        self.add_obj_value = 10 # add intention once detected again 
        self.minus_obj_value = -10
        self.add_class_value = 7
        self.fading_value = 2 # let history fade away 

        #polarize lidar from camera params
        self.angle_of_view = np.radians(70) 
        self.radialscan_num = 100
        self.lidarfromcam_polarcoord = [-1] * self.radialscan_num 
        self.angle_step = self.angle_of_view / (self.radialscan_num - 1) 

    def add_lidar_result(self, msg:list[Point]):
        assert isinstance(msg, list) and all(isinstance(p, Point) for p in msg), "msg must be a list of Point objects"

        for point in msg:
            ox, oy, tx, ty, gx, gy, rx, ry = self.original_transform_grid_relative(point = point)

            # check range
            if 0 <= gx < self.width and 0 <= gy < self.height:
                self.cache_map[gy, gx] = 100
        
        # self.save_map_local(self.cache_map, "cache")

    def add_pointcloud_result_v2(self, msg: list[PointcloudDetectedObj]):
        for o in msg:
            assert isinstance(o, PointcloudDetectedObj), f"Each element in msg should be of type PointcloudDetectedObj, but got {type(o)}."
            assert o.category in ['box', 'item'], f"Invalid category {o.category}. It should be either 'box' or 'item'."
            ox, oy, tx, ty, gx, gy, rx, ry = self.original_transform_grid_relative(position = o.pose.pose.position)

            # check range
            if 0 <= gx < self.cache_map.shape[1] and 0 <= gy < self.cache_map.shape[0]:
                self.pointcloud_msg_list.append(o) 
            else:
                # print(f"Invalid index: ({grid_x}, {grid_y}), grid point out of map")
                pass

    def add_yolo_result_v2(self, msg):
        if msg is None: 
            self.get_logger().error("yolo msg is none, wtf?")
            return  

        if msg.objects[0].category != "no_detection":
            self.yolo_category_list = msg.category_list
        else:
            return

        msg = msg.objects #a list, so it align with the structure before

        height_good_objs = []
        # Step 1: Filter out objects based on z position threshold
        for i, m in enumerate(msg):
            if m.center_point.pose.position.z > self.yolopoint_mapping_height_threshold:
                continue  # Skip objects above the threshold
            height_good_objs.append(m)

        jump_overlap = []  # List to track overlaps
        no_overlap_objs = []   # List to store objects after overlap handling

        # Step 2: Handle overlap between valid objects
        for i, valid_o in enumerate(height_good_objs):
            if i not in jump_overlap:
                overlap_found = False

                for j, other_o in enumerate(height_good_objs):
                    if i != j and valid_o.category == other_o.category:
                        # Calculate distance between centers
                        distance = np.sqrt((valid_o.center_point.pose.position.x - other_o.center_point.pose.position.x) ** 2 +
                                           (valid_o.center_point.pose.position.y - other_o.center_point.pose.position.y) ** 2)
                        
                        if distance < self.yolopoint_mapping_remove_overlap_threshold:
                            # Calculate the midpoint between the two objects
                            mid_x = (valid_o.center_point.pose.position.x + other_o.center_point.pose.position.x) / 2
                            mid_y = (valid_o.center_point.pose.position.y + other_o.center_point.pose.position.y) / 2

                            # Update the center point of valid_o to the midpoint
                            valid_o.center_point.pose.position.x = mid_x
                            valid_o.center_point.pose.position.y = mid_y
                            no_overlap_objs.append(valid_o) 
                            jump_overlap.append(j)  # Mark the other object for removal
                            overlap_found = True
                            break
                
                if not overlap_found:
                    no_overlap_objs.append(valid_o)  # Add valid_o to final list if no overlap found

        # Step 3: Update cache map for each valid object
        for valid_o in no_overlap_objs:
            self.yolo_msg_list.append(valid_o)

    def add_lidarfromcam_result(self, map_points:list[Point]):
        if map_points == None:
            self.get_logger().error("for some reason, result from 'lidarfromcam' is empty, wtf?")
            return
        
        for p in map_points:
            assert isinstance(p, Point), f"Each element in points should be of type Point, but got {type(p)}."
            ox, oy, tx, ty, gx, gy, rx, ry = self.original_transform_grid_relative(point = p)
            
            if 0 <= gx < self.cache_map.shape[1] and 0 <= gy < self.cache_map.shape[0]:
                self.cache_map[gy, gx] = 99
                # if self.cache_map[gy, gx] == 0:
                    # self.cache_map[gy, gx] = 99
            else:
                # print(f"Invalid index: ({gx}, {gy}), out of map")
                pass

            distance = math.sqrt(rx**2 + ry**2)  # Radius (distance to the center)
            angle = math.atan2(ry, rx)  # Angle of the point in radians relative to the robot

            min_angle = -self.angle_of_view / 2.0  # The leftmost angle of the scan
            max_angle = self.angle_of_view / 2.0   # The rightmost angle of the scan
            angle_step = (max_angle - min_angle) / (self.radialscan_num - 1)  # Step between angles

            scan_angle_index = int((angle - min_angle) / angle_step)
            if 0 <= scan_angle_index < self.radialscan_num:
                if distance < self.lidarfromcam_polarcoord[scan_angle_index]:
                    self.lidarfromcam_polarcoord[scan_angle_index] = distance

        # print("Lidar Polar Coordinates: ", self.lidarfromcam_polarcoord)

        self.save_map_local(self.cache_map, "cache")       


    def check_if_visiable(self, point):
        """
        input real position in map frame 
        return bool
        """
        rx = point.x - self.robot_pose.position.x 
        ry = point.y - self.robot_pose.position.y 

        distance = math.sqrt(rx**2 + ry**2)
        angle = math.atan2(ry, rx)

        min_angle = -self.angle_of_view / 2.0  
        max_angle = self.angle_of_view / 2.0  
        scan_angle_index = int((angle - min_angle) / self.angle_step)
        
        scan_distance = self.lidarfromcam_polarcoord[scan_angle_index]

        if scan_distance == -1 or distance < scan_distance + self.tolerance_between_obj_and_lidarfromcam:
            print(f"Point {point} is visible.")
            return True
        else: 
            return False

    def add_intention_v2(self, index, add_class_index, add_value, newposestamped = None):
        """
        find point by index, and change value for intention
        remember that intention should be more than points on the map
        """
        if index == -1 :
            max_index = max([item['index'] for item in self.intentionpoint_dictlist], default=-1)
            new_index = max_index + 1

            new_intention = [0] * 5
            new_intention[add_class_index] += add_value

            new_dict = {
                'index': new_index,
                'point': newposestamped, 
                'intention': new_intention  
            }
            self.intentionpoint_dictlist.append(new_dict)
            # print(f"new intention in position x{str(newposestamped.pose.position.x)}, y{str(newposestamped.pose.position.y)}")
            return 
        
        row = next((item for item in self.intentionpoint_dictlist if item['index'] == index), None)
        if row == None:
            self.get_logger().info(f"point with index {index} not found ")
        else:
            row["intention"][add_class_index] += add_value
            # print(f"add intention for point index {index} with class {add_class_index} and value {add_value}")

    def key_strategy(self):
        # add intention and move if point existed before, for points that are not detected, decrease intention, for points newly detected increase intention
        mappoint_dictlist_unoperated = self.mappoint_dictlist
        for msg in self.pointcloud_msg_list:
            matched_point_dict = None
            matched_intention_dict = None
            cache_distance = 0.2 

            for dict in self.mappoint_dictlist:
                distance = np.sqrt((msg.pose.pose.position.x - dict['point'].pose.position.x)**2 + 
                                (msg.pose.pose.position.y - dict['point'].pose.position.y)**2)
                
                if distance < self.same_obj_existed_before:
                    if distance < cache_distance:
                        cache_distance = distance
                        matched_point_dict = dict  # closest point and also inside threshold 

            if matched_point_dict is not None:# existed before 
                mappoint_dictlist_unoperated.remove(matched_point_dict)
                
                # print("1")
                self.add_intention_v2(matched_point_dict['index'], 0, self.add_obj_value)
                # update the mappoint 
                for dict in self.mappoint_dictlist:
                    if dict['index'] == matched_point_dict['index']:
                        dict.point = msg.pose
                        break  

            else: # new point detected                 
                for dict in self.intentionpoint_dictlist:
                    distance = np.sqrt((msg.pose.pose.position.x - dict['point'].pose.position.x)**2 + 
                                (msg.pose.pose.position.y - dict['point'].pose.position.y)**2)
                    if distance < self.same_obj_existed_before:
                        if distance < cache_distance:
                            cache_distance = distance
                            matched_intention_dict = dict  # closest point and also inside threshold 

                if matched_intention_dict == None:
                    # print("2")
                    self.add_intention_v2(-1, 0, self.add_class_value, msg.pose)
                else:
                    # print("3")
                    self.add_intention_v2(dict['index'], 0, self.add_obj_value)
                
        for dict in mappoint_dictlist_unoperated: # old points not detected 
            if self.check_if_visiable(dict['point'].pose.position):
                # print("4")
                self.add_intention_v2(dict['index'], 0, self.minus_obj_value)
            else:
                pass

        #align yolo to rgbd 
        for msg in self.yolo_msg_list:
            closest_intention = None
            closest_distance = self.align_yolo_to_pc_radius_threshold

            for intentiondict in self.intentionpoint_dictlist:
                distance = np.sqrt((msg.center_point.pose.position.x - intentiondict['point'].pose.position.x)**2 + 
                                (msg.center_point.pose.position.y - intentiondict['point'].pose.position.y)**2)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_intention = intentiondict  
                    print("yolo had a chance to be aligned!")

            if closest_intention is not None:
                # print("5")
                self.add_intention_v2(closest_intention['index'], msg.category_num+1, self.add_class_value) ##########

    def update_map_with_intention_v2(self, fading = True):
        # print("mappoint dict list")
        # print(self.mappoint_dictlist)        
        # print("intention dict list")
        # print(self.intentionpoint_dictlist)

        for rowindex, dict in enumerate(self.intentionpoint_dictlist):
            # add champing 
            dict["intention"] = [max(min(val, self.intention_value_range), -self.intention_value_range) for val in dict["intention"]]

            # add fading 
            if self.fading:
                dict["intention"][1:] = [val - self.fading_value for val in dict["intention"][1:]]

            # add new point to map from index 
            if dict["intention"][0] > self.have_obj_threshold:
                max_intention = max(dict["intention"][1:])
                if max_intention > self.have_class_threshold:
                    category = self.yolo_category_list[dict["intention"][1:].index(max_intention) - 1]  

                    max_index = max([item['index'] for item in self.mappoint_dictlist], default=-1)
                    new_index = max_index + 1
                    new_dict = {
                        'index': new_index,
                        'point': dict["point"],  
                        'class': category 
                    }
                    self.mappoint_dictlist.append(new_dict)

            # Step 4: Reassign the index in intentionpoint_dictlist and update the corresponding mappoint_dictlist
            indexbefore = dict['index']
            dict['index'] = rowindex  
            # Now update the corresponding dict in mappoint_dictlist
            for mappoint in self.mappoint_dictlist:
                if mappoint['index'] == indexbefore:
                    mappoint['index'] = rowindex  

        print("mappoint dict list")
        print(self.mappoint_dictlist)        
        print("intention dict list")
        print(self.intentionpoint_dictlist)

    def re_init(self):
        self.cache_map.fill(-1)
        self.lidarfromcam_polarcoord = [-1]* self.radialscan_num
        self.pointcloud_msg_list = []
        self.yolo_msg_list = []

    def get_robot_pose(self):
        self.robot_pose.position.x = 50 
        self.robot_pose.position.y = 50
        self.robot_pose.orientation.z = 0

    def original_transform_grid_relative(self, point = None, position = None):
        if point != None:
            original_point_x = point.x #point on map frame
            original_point_y = point.y
        elif position != None:
            original_point_x = position.x #point on map frame
            original_point_y = position.y            
        else:
            self.get_logger().error("invalid input: no point or position")

        transformed_point_x = original_point_x - self.origin[0] #point on gridmap coordimate
        transformed_point_y = original_point_y - self.origin[1]
        grid_x = int(transformed_point_x / self.resolution)
        grid_y = int(transformed_point_y / self.resolution)
        relative_x = original_point_x - self.robot_pose.position.x
        relative_y = original_point_y - self.robot_pose.position.y
        ox, oy, tx, ty, gx, gy, rx, ry = original_point_x, original_point_y, transformed_point_x, transformed_point_y, grid_x, grid_y, relative_x, relative_y
        return ox, oy, tx, ty, gx, gy, rx, ry

#just for visualize 
    def publish_map(self,map):
        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = 'map'  

        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.resolution = self.resolution

        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.orientation.w = 1.0 

        msg.data = map.flatten().tolist()

        self.publisher.publish(msg)
        self.get_logger().info('Published grid map')

    def save_map_local(self, map, name):
        map_image = np.array(map, dtype=np.float32)  # Convert to float32 to handle negative values
        # Step 2: Create a blank image to store the color-mapped data
        color_map = np.zeros((self.height, self.width, 3), dtype=np.uint8)  # RGB image
        # Step 3: Manually set colors based on map values
        for i in range(self.height):
            for j in range(self.width):
                value = map_image[i, j]
                if value == -1:
                    # Unknown/Empty space (black)
                    color_map[i, j] = [0, 0, 0]  # Black
                elif value == 0:
                    # Free space (white)
                    color_map[i, j] = [255, 255, 255]  # White
                elif value == 100:
                    # Occupied space (red)
                    color_map[i, j] = [0, 0, 255]  # Red
                elif value == 99:
                    color_map[i, j] = [255, 0, 0]                
                elif value == 20:
                    color_map[i, j] = [0, 255, 0]  # Red
                elif value > 0 and value < 100:
                    # Intermediate values (green to yellow gradient)
                    green_value = int((value / 100) * 255)
                    color_map[i, j] = [0, green_value, 255 - green_value]  # Green to Yellow gradient
        color_map = np.flipud(color_map)
        # Step 4: Save the image to a file
        output_dir = "/home/sleepy/robp-group7-sleepy/yolo"  # Change this to your desired directory
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        output_file = os.path.join(output_dir, name+".png")
        cv2.imwrite(output_file, color_map)

        self.get_logger().info(f'Saved map image to {output_file}')

#abandoned
    def add_explored_area(self,min_dist = 0.25, max_dist = 1):
        angle_of_view_rad = self.angle_of_view/2

        relative_max_dist = max_dist/self.resolution
        relative_min_dist = min_dist/self.resolution

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_theta = self.robot_pose.orientation.z
        
        x_coords, y_coords = np.meshgrid(np.arange(self.width), np.arange(self.height))
        
        # move coord to get relative normalized position(index actually)
        relative_x = x_coords - robot_x
        relative_y = y_coords - robot_y
        
        distances = np.sqrt(relative_x ** 2 + relative_y ** 2)
        angles = np.arctan2(relative_y, relative_x) - robot_theta ###########is this safe? maybe?
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        
        mask_within_range = (distances >= relative_min_dist) & (distances <= relative_max_dist)
        mask_within_angle = np.abs(angles) <= angle_of_view_rad
        combined_mask = mask_within_range & mask_within_angle
        
        self.cache_map[combined_mask & (self.cache_map == -1)] = 0

        self.save_map_local(self.cache_map, "cache")

        return self.cache_map

    def add_intention(self, point, add_class_index, add_value, remove_point = None, fading = False):
        """
        params:
            add class index: where to add value on the intention map, there are 5 index in the list, representing there is sth, there is class 1, there is class 2 ...., there is class 4
            add value: the value to add 
        """
        if remove_point != None:
            rx, ry = remove_point
            self.intention_map[rx,ry] = np.array([0, 0, 0, 0, 0])
        if fading:
            self.intention_map -= self.fading_value  
        
        x, y = point
        self.intention_map[y, x, add_class_index] += add_value

        self.intention_map[y, x, add_class_index] = np.clip(self.intention_map[y, x, add_class_index], 
                                                      -self.intention_value_range, 
                                                      self.intention_value_range)  

    def add_yolo_result(self, msg: list[YoloClassifiedObject]):
        height_good_objs = []
        if msg is None: 
            return  
        
        # Step 1: Filter out objects based on z position threshold
        for i, m in enumerate(msg):
            if m.center_point.pose.position.z > self.yolopoint_mapping_height_threshold:
                continue  # Skip objects above the threshold
            height_good_objs.append(m)

        jump_overlap = []  # List to track overlaps
        no_overlap_objs = []   # List to store objects after overlap handling

        # Step 2: Handle overlap between valid objects
        for i, valid_o in enumerate(height_good_objs):
            if i not in jump_overlap:
                overlap_found = False

                for j, other_o in enumerate(height_good_objs):
                    if i != j and valid_o.category == other_o.category:
                        # Calculate distance between centers
                        distance = np.sqrt((valid_o.center_point.pose.position.x - other_o.center_point.pose.position.x) ** 2 +
                                           (valid_o.center_point.pose.position.y - other_o.center_point.pose.position.y) ** 2)
                        
                        if distance < self.yolopoint_mapping_remove_overlap_threshold:
                            # Calculate the midpoint between the two objects
                            mid_x = (valid_o.center_point.pose.position.x + other_o.center_point.pose.position.x) / 2
                            mid_y = (valid_o.center_point.pose.position.y + other_o.center_point.pose.position.y) / 2

                            # Update the center point of valid_o to the midpoint
                            valid_o.center_point.pose.position.x = mid_x
                            valid_o.center_point.pose.position.y = mid_y
                            no_overlap_objs.append(valid_o) 
                            jump_overlap.append(j)  # Mark the other object for removal
                            overlap_found = True
                            break
                
                if not overlap_found:
                    no_overlap_objs.append(valid_o)  # Add valid_o to final list if no overlap found

        # Step 3: Update cache map for each valid object
        for valid_o in no_overlap_objs:
            x, y = valid_o.center_point.pose.position.x, valid_o.center_point.pose.position.y
            x = x - self.origin[0]
            y = y - self.origin[1]
            nearby_points = self.find_value_in_range(self.cache_map, [20], (x, y), self.align_yolo_to_pc_radius_threshold)
            if nearby_points:
                # If nearby points with value 20 are found, add intention
                self.add_intention(nearby_points[0], 1, self.add_class_value, fading=True)

    def add_pointcloud_result(self, msg: list[PointcloudDetectedObj]):
        for o in msg:
            assert isinstance(o, PointcloudDetectedObj), f"Each element in msg should be of type PointcloudDetectedObj, but got {type(o)}."
            assert o.category in ['box', 'item'], f"Invalid category {o.category}. It should be either 'box' or 'item'."
            x = o.pose.pose.position.x
            y = o.pose.pose.position.y
            grid_x = int((x - self.origin[0]) / self.resolution)
            grid_y = int((y - self.origin[1]) / self.resolution)

            # Check if the position is within the map boundaries
            if 0 <= grid_x < self.cache_map.shape[1] and 0 <= grid_y < self.cache_map.shape[0]:
                if self.cache_map[grid_y, grid_x] == 0:  # If the grid cell empty and explored
                    nearby_values = self.find_value_in_range(self.cache_map, [100], (grid_x, grid_y), self.avoid_obj_threshold)#check if there are obstacles to mislead the pointcloud detection
                    if not nearby_values:
                        self.cache_map[grid_y, grid_x] = 20  # Mark the cache map
                        self.add_intention((grid_x, grid_y), 1, self.add_obj_value)##############################
                else:
                    print("Position already occupied")
            else:
                print(f"Invalid index: ({grid_x}, {grid_y})")
        
        self.save_map_local(self.cache_map, "cache")

    def find_value_in_range(self, map, values, center_point, radius):
        assert isinstance(map, np.ndarray), "The map should be a NumPy array."
        assert isinstance(values, list), "Values should be a list or set."

        radius_in_cells = int(radius / self.resolution)

        center_x, center_y = center_point
        result_indices = []
        # Loop through the grid and check for the value within the circular region
        for x in range(max(0, int(center_x - radius_in_cells)), min(self.width, int(center_x + radius_in_cells + 1))):
            for y in range(max(0, int(center_y - radius_in_cells)), min(self.height, int(center_y + radius_in_cells + 1))):
                # Calculate the Euclidean distance from the center point
                distance = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                if distance <= radius_in_cells and map[y, x] in values:
                    result_indices.append((distance, (x, y))) 

        result_indices.sort(key=lambda item: item[0])

        return [item[1] for item in result_indices]

    def update_map_with_intention(self):
        for x in range(self.width):
            for y in range(self.height):
                intention = self.intention_map[y, x]

                if intention[0] > self.have_obj_threshold:
                    
                    class_over_threshold = [i for i in range(1, 5) if intention[i] > self.have_class_threshold]
                    
                    if not class_over_threshold:
                        self.grid_map[y, x] = 20
                    else:
                        max_class_index = max(class_over_threshold, key=lambda i: intention[i])
                        self.grid_map[y, x] = max_class_index + 20
                else:
                    continue


def main(args=None):
    rclpy.init(args=args)
    node = MapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
