import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from core_interfaces.srv import GridCreator
import numpy as np
from shapely.geometry import Point, Polygon

import os
class grid_map_creator(Node):

    def __init__(self):
        super().__init__('grid_map_creator')
        print("inne grid_map_creator 7")
        self.grid_gen_srv = self.create_service(GridCreator, "fill_in_workspace", self.fill_in_workspace)
        self.origin = None
        self.resolution = None
        self.padding = None
        self.grid = None
        self.workspace_file = None
        

    def fill_in_workspace(self, request, response):
        self.padding = request.padding
        self.resolution = request.resolution
        file = request.file_name
        read_start_row = 0 
        file_path = os.path.expanduser(file)
        while True:
            try:
                data = np.loadtxt(file_path, delimiter="\t", skiprows=read_start_row)
                break
            except ValueError:
                read_start_row += 1

        max_x = np.max(data[:,0])
        min_x = np.min(data[:,0])
        max_y = np.max(data[:,1])
        min_y = np.min(data[:,1])

        nr_cells_x_dir = np.ceil(np.abs(max_x-min_x)/self.resolution)
        nr_cells_y_dir = np.ceil(np.abs(max_y-min_y)/self.resolution)

        polygon = Polygon(data)

        inner_polygon = polygon.buffer(-self.padding, join_style=2)


        self.grid = np.zeros((int(nr_cells_y_dir), int(nr_cells_x_dir)), dtype=np.int8)
        self.grid -=1
        rows = self.grid.shape[0]
        cols = self.grid.shape[1]
        rest_x = (np.abs(max_x-min_x)-(cols-1)*self.resolution)/(cols-1)
        rest_y = (np.abs(max_y-min_y)-(rows-1)*self.resolution)/(rows-1)
        for i in range(rows):
            for j in range(cols):

                p = Point(min_x+0.001+j*(self.resolution+rest_x), min_y+0.001+i*(self.resolution+rest_y))
                
                if inner_polygon.contains(p):
                    self.grid[i, j] = 0

        self.origin = [-min_x/self.resolution, -min_y/self.resolution]
        msg = self.publish_grid_map()
        response.grid = msg
        print("Grid shape")
        print(self.grid.shape)
        return response

    def publish_grid_map(self):

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" 

        msg.info.resolution = float(self.resolution)
        msg.info.width = self.grid.shape[1]
        msg.info.height = self.grid.shape[0]

        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  

        msg.data = self.grid.flatten().tolist()

        return msg



def main():
    rclpy.init()
    node = grid_map_creator()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()





