import numpy as np
import os
from random import randint

class Gridmap:
    def __init__(self, resolution, x_min, x_max, y_min, y_max, z_min, z_max):
        self.set_parameters(resolution, x_min, x_max, y_min, y_max, z_min, z_max)
        self.gridmap = self.construct_gridmap()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.location = current_dir + "/gridmaps/"

    def set_parameters(self, resolution, x_min, x_max, y_min, y_max, z_min, z_max): 
        self.resolution = resolution
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        self.nx = int((self.x_max - self.x_min) / self.resolution) 
        self.ny = int((self.y_max - self.y_min) / self.resolution) 
        self.nz = int((self.z_max - self.z_min) / self.resolution) 

    def save_gridmap(self):
        simple_id = str(randint(100, 999))  # 3-digit random number
        filename = f"{self.x_min}_{self.x_max}_{self.y_min}_{self.y_max}_{self.z_min}_{self.z_max}_{self.resolution}_{simple_id}.npy"
        np.save(self.location + filename, self.gridmap)

    def load_gridmap(self, id):

        files_in_dir = os.listdir(self.location) 
        for file in files_in_dir: 
            if id in file:
                x_min, x_max, y_min, y_max, z_min, z_max, resolution, uuid = file.split(".npy")[0].split("_")
                self.set_parameters(float(resolution), 
                                    float(x_min), 
                                    float(x_max), 
                                    float(y_min), 
                                    float(y_max), 
                                    float(z_min), 
                                    float(z_max))
                self.gridmap = np.load(self.location + file)
                return
        raise ValueError(f"Gridmap with id {id} not found")

    def construct_gridmap(self):
        gridmap = np.zeros((self.nx, self.ny, self.nz))
        return gridmap

    def idx_to_xyz(self, idx):
        """ 
        Convert gridmap index to cartesian coordinates
        Returns the center of the voxel
        """
        x = idx[0] * self.resolution + self.x_min + self.resolution/2
        y = idx[1] * self.resolution + self.y_min + self.resolution/2
        z = idx[2] * self.resolution + self.z_min + self.resolution/2
        return x, y, z
    
    def xyz_to_idx(self, x, y, z):
        """ 
        Convert cartesian coordinates to gridmap index
        rounded to closest voxel
        """
        idx = np.array([(x - self.x_min) / self.resolution, (y - self.y_min) / self.resolution, (z - self.z_min) / self.resolution])
        return np.floor(idx)

    def add_box_obstacle(self, x, y, z, l, w, h):
        """ 
        Add a box obstacle to the gridmap. 
        x, y, z: center of the box
        l, w, h: dimensions of the box
        """
        # Convert the box to gridmap indices
        idx_min = self.xyz_to_idx(x - l/2, y - w/2, z - h/2) 
        idx_max = self.xyz_to_idx(x + l/2, y + w/2, z + h/2)
        # Clip the indices to the gridmap bounds
        idx_min = np.clip(idx_min, 0, self.nx - 1)
        idx_max = np.clip(idx_max, 0, self.ny - 1)
        idx_max = np.clip(idx_max, 0, self.nz - 1)
        self.gridmap[int(idx_min[0]):int(idx_max[0]) + 1, int(idx_min[1]):int(idx_max[1]) + 1, int(idx_min[2]):int(idx_max[2]) + 1] = 1

    def collision_free(self, x, y, z):
        """ 
        Check if the point is collision free with a sphere of radius r
        """
        X_idx = self.xyz_to_idx(x, y, z) 
        if X_idx[0] < 0 or X_idx[0] >= self.nx or X_idx[1] < 0 or X_idx[1] >= self.ny or X_idx[2] < 0 or X_idx[2] >= self.nz:
            return False
        occupied_idx = np.where(self.gridmap > 0) 
        for i in range(len(occupied_idx[0])):
            idx_x = occupied_idx[0][i]
            idx_y = occupied_idx[1][i]
            idx_z = occupied_idx[2][i]
            occupied_x, occupied_y, occupied_z = self.idx_to_xyz([idx_x, idx_y, idx_z])
            if np.linalg.norm(np.array([x, y, z]) - np.array([occupied_x, occupied_y, occupied_z])) < self.resolution*2**(1/2):
                return False
        return True

    def get_occupied_positions(self):
        occupied = np.where(self.gridmap == 1)
        positions = np.vstack([
            occupied[0] * self.resolution + self.x_min,
            occupied[1] * self.resolution + self.y_min,
            occupied[2] * self.resolution + self.z_min
        ]).T
        return positions

if __name__ == "__main__":
    gridmap = Gridmap(resolution=0.05, x_min=-0.1, x_max=0.4, y_min=-.4, y_max=.2, z_min=0, z_max=0.5)
    gridmap.add_box_obstacle(-0.15, 0, 0, 0.4, 0.3, 0.2)
    gridmap.add_box_obstacle(0, 0.1, 0.2, 0.1, 0.1, 0.05)
    # gridmap.save_gridmap()
