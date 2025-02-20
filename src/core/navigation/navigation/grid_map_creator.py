import rclpy
from rclpy.node import Node
from navigation.polygon import LocalPolygon
import numpy as np

import os
class grid_map_creator(Node):

    def __init__(self):
        super().__init__('grid_map_creator')
        print("inne grid_map_creator 3")
        self.origin = None
        self.resolution = 5
        self.padding = 10
        self.grid = None
        self.workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_2.tsv"
        self.fill_in_workspace()
        self.grid_pub = self.create_publisher(np.ndarray, '/occupancy_grid', 10)
        self.grid_pub.publish(self.grid)


    def fill_in_workspace(self):

        read_start_row = 0 
        file_path = os.path.expanduser(self.workspace_file)
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

        cell_nr_x_dir = int(np.round((max_x-min_x)/self.resolution, 0))
        cell_nr_y_dir = int(np.round((max_y-min_y)/self.resolution, 0))

        self.grid = np.zeros((cell_nr_y_dir, cell_nr_x_dir))

        self.origin = (-min_x, -min_y)
        for row in range(len(data)):
            for column in range(2):
                data[row, column] = data[row, column]-self.padding*data[row, column]/np.abs(data[row, column])

        polygon = LocalPolygon(data)

        for x in range(int(min_x), int(max_x)):
            for y in range(int(min_y), int(max_y)):
                if polygon.is_internal(np.array([x, y])) == False:
                    self.grid[int((y-min_y)/self.resolution), int((x-min_x)/self.resolution)] = -1

        np.save("saved_grid.npy", self.grid)

def main():
    rclpy.init()
    node = grid_map_creator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()





