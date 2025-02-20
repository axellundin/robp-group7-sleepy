
from math import cos, sin, atan2, fabs
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from map_msgs.msg import OccupancyGridUpdate
import rclpy
from rclpy.node import Node



class Grid_map_filler(Node):
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius):
        super().__init__('grid_map_filler')
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.grid = None
        self.grid_sub = self.create_subscription(
            np.ndarray,
            "/occupancy_grid",
            self.start_grid_callback,
            10
        )
        self.grid_sub

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_listener_callback,
            10
        )
        self.lidar_sub

    def start_grid_callbeck(self, msg):
        self.grid = msg

    def lidar_listener_callback(self, scan):
        if self.grid == None:
            return
        self.update_map(scan)
        

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """

        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False
    
    def transformation(self, x, y, z, yaw):
        trans_matrix = np.array([[np.cos(yaw),-np.sin(yaw), 0, x+3.75],[np.sin(yaw),np.cos(yaw), 0, y+3.75],[0, 0, 1, z]])
        return trans_matrix
    

    def update_map(self, grid_map, pose, scan):

        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        wr = pose.pose.orientation.w
        xr = pose.pose.orientation.x
        yr = pose.pose.orientation.y
        zr = pose.pose.orientation.z

        resolution = grid_map.get_resolution()
        

        #Creates the transformation matrix from the robot to the base
        yaw=np.arctan2(2*(wr*zr+xr*yr),1-2*(yr**2+zr**2))
        trans_matrix = self.transformation(x, y, z, yaw)

        x_max = 0
        x_min= grid_map.get_width()
        y_max = 0
        y_min= grid_map.get_height()

        grid = np.full((grid_map.get_height(), grid_map.get_width()),-1)
        grid_check = np.full((grid_map.get_height(), grid_map.get_width()),-1)
        

        for i in range(len(scan.ranges)):
            
            if not(scan.ranges[i]<=scan.range_min) and not(scan.ranges[i]>=scan.range_max):
                x_obs =  scan.ranges[i]*np.cos(i*scan.angle_increment)
                y_obs =  scan.ranges[i]*np.sin(i*scan.angle_increment)
                position_obs = np.dot(trans_matrix, [[x_obs],[y_obs],[0],[1]])
                position_bot = np.dot(trans_matrix, [[0],[0],[0],[1]])
                x_obs_new = int(position_obs[0,0]//resolution)
                y_obs_new = int(position_obs[1,0]//resolution)
                
                free_space = self.raytrace((int(position_bot[0,0]//resolution), int(position_bot[1,0]//resolution)),(x_obs_new, y_obs_new))


                for point in free_space:

                    if grid_map.__getitem__((point[0], point[1])) != self.free_space and grid_check[point[1],point[0]]!=self.occupied_space:
                        #Update map with free space
                        self.add_to_map(grid_map, point[0], point[1], self.free_space)

                        #Update grid with free space
                        grid[point[1],point[0]] = self.free_space
                        grid_check[point[1],point[0]] = self.free_space
                        #Change max and min, x and y value
                        if point[0] < x_min:
                            x_min = point[0]
                        if point[0] > x_max:
                            x_max = point[0]
                        if point[1] < y_min:
                            y_min = point[1]
                        if point[1] > y_max:
                            y_max = point[1]
                        
                if grid_map.__getitem__((x_obs_new, y_obs_new)) != self.occupied_space:
                    #Update map with occupied space
                    self.add_to_map(grid_map, x_obs_new, y_obs_new, self.occupied_space)

                    #Update grid with occupied space
                    grid[y_obs_new,x_obs_new] = self.occupied_space
                    
                    #Change max and min, x and y value
                    if x_obs_new < x_min:
                        x_min = x_obs_new
                    if x_obs_new > x_max:
                        x_max = x_obs_new
                    if y_obs_new < y_min:
                        y_min = y_obs_new
                    if y_obs_new > y_max:
                        y_max = y_obs_new
                grid_check[y_obs_new,x_obs_new] = self.occupied_space
        


 
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """
        """
        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        """


        """
        Fill in your solution here
        """


        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_min
        # Maximum x index - minimum x index + 1
        update.width = x_max - x_min + 1
        # Maximum y index - minimum y index + 1
        update.height = y_max - y_min + 1
        # The map data inside the rectangle, in row-major order.
        rectangle = grid[y_min:y_max+1, x_min:x_max+1]
        rectangle_array = rectangle.reshape(-1)
        update.data = rectangle_array

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        
        y_add = [5,
                 4,4,4,4,4,4,4,
                 3,3,3,3,3,3,3,3,3,
                 2,2,2,2,2,2,2,2,2,
                 1,1,1,1,1,1,1,1,1,
                 0,0,0,0,0,0,0,0,0,0,0,
                 -1,-1,-1,-1,-1,-1,-1,-1,-1,
                 -2,-2,-2,-2,-2,-2,-2,-2,-2,
                 -3,-3,-3,-3,-3,-3,-3,-3,-3,
                 -4,-4,-4,-4,-4,-4,-4,
                 -5,]
        x_add = [0,
                 -3,-2,-1,0,1,2,3,
                 -4,-3,-2,-1,0,1,2,3,4,
                 -4,-3,-2,-1,0,1,2,3,4,
                 -4,-3,-2,-1,0,1,2,3,4,
                 -5,-4,-3,-2,-1,0,1,2,3,4,5,
                 -4,-3,-2,-1,0,1,2,3,4,
                 -4,-3,-2,-1,0,1,2,3,4,
                 -4,-3,-2,-1,0,1,2,3,4,
                 -3,-2,-1,0,1,2,3,
                 0]
        for rad in range(300):
            for collumn in range(300):

                if grid_map.__getitem__((collumn, rad)) == self.occupied_space:
                    for i in range(len(y_add)):
                        if grid_map.__getitem__((collumn+x_add[i], rad+y_add[i])) != self.occupied_space:
                            self.add_to_map(grid_map, collumn+x_add[i], rad+y_add[i], self.c_space)
                                                        

        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """
        
        # Return the inflated map
        return grid_map

def main():
    rclpy.init()
    node = Grid_map_filler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
