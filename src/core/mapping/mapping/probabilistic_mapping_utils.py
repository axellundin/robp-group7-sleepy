import numpy as np 
import cv2
import time

class ProbabilisticMapper:
    def __init__(self, map_size: np.ndarray, resolution: float, origin: np.ndarray):
        self.L_0 = 0  
        self.L_OCC = 30
        self.L_FREE = -0.5 # -2

        self.alpha = 0.1
        self.beta = 0.3
        self.z_max = 5
        self.origin = origin

        self.map_width = map_size[0] 
        self.map_height = map_size[1] 
        self.should_downsample = True 
        self.resolution = resolution # meters 
        self.downsampled_resolution = resolution
        self.downsampled_width = int(self.map_width * self.resolution / self.downsampled_resolution) 
        self.downsampled_height = int(self.map_height * self.resolution / self.downsampled_resolution) 
        self.downsampled_origin = np.array([int(x * self.resolution / self.downsampled_resolution) for x in self.origin])
        self.log_odds = self.L_0 * np.ones((self.downsampled_width, self.downsampled_height)) if self.should_downsample else self.L_0 * np.ones((self.map_width, self.map_height))
        self.map = self.recover_probabilities(self.upsample_map(self.log_odds))
        self.inflated_map = self.map 
        self.inflated_map_planning = self.map 
        self.map_binary = self.map > 30

    def from_pos_to_idx(self, x: np.ndarray, resolution=None, origin=None): 
        """ Convert position to index, from center of the cell to the center of the cell"""
        if resolution is None: 
            resolution = self.resolution
        if origin is None:  
            origin = self.origin
        return int(np.round((x[0]/resolution)+origin[0],0)), int(np.round((x[1]/resolution)+origin[1],0))
    
    def from_idx_to_pos(self, idx: np.ndarray): 
        """ Convert index to position, from center of the cell to the center of the cell"""
        return (idx + 0.5 * np.ones(idx.shape)) * self.resolution + self.origin

    def median_filter_1d(self, signal, kernel_size=3):
        """
        Apply a 1D median filter to a signal.
        
        Parameters:
            signal (np.ndarray): 1D input array
            kernel_size (int): Size of the median filter window (must be odd)
        
        Returns:
            np.ndarray: Filtered signal
        """
        if kernel_size % 2 == 0:
            raise ValueError("kernel_size must be odd")

        pad_width = kernel_size // 2
        padded_signal = np.pad(signal, pad_width, mode='edge')
        
        filtered = np.empty_like(signal)
        for i in range(len(signal)):
            window = padded_signal[i:i + kernel_size]
            filtered[i] = np.median(window)

        return filtered

    def inverse_sensor_model_old(self, measurements: np.ndarray, x: np.ndarray):
        """ Assuming x = [x,y,theta], with theta in radians """
        measurements = self.median_filter_1d(measurements, 3)
        idx = self.from_pos_to_idx(x)
        idx_x = idx[0]
        idx_y = idx[1]

        x_dist = (np.tile(np.arange(self.map_width), (self.map_height, 1)) - idx_y * np.ones((self.map_height, self.map_width)) ) * self.resolution
        y_dist = (np.tile(np.arange(self.map_height), (self.map_width, 1)).T - idx_x * np.ones((self.map_height, self.map_width)) ) * self.resolution

        R_map = np.sqrt(np.sum(np.array([x_dist, y_dist]) ** 2, axis=0))
        print("theta: ", x[2])
        phi = np.arctan2(x_dist, y_dist) - x[2] * np.ones((self.map_height, self.map_width))
        phi = np.mod(phi + np.pi, 2*np.pi) - np.pi

        # print(f"phi: {phi[idx_x-3:idx_x+3, idx_y-3:idx_y+3]}")
        # Convert measurement angles from degrees to radians for consistent comparison
        measurement_angles = np.arange(len(measurements)) * (np.pi / 180) - np.pi
        Phi = phi[:,:,np.newaxis] #  * np.ones((1,len(measurements)))
        Theta = np.ones((self.map_height, self.map_width))[:,:,np.newaxis] * measurement_angles
        print(f"Shape of Phi: {Phi.shape}")
        print(f"Shape of Theta: {Theta.shape}")
        delta_angle = np.angle(np.exp(1j * ( Phi - Theta + np.pi))) + np.pi
        # delta_angle = Phi - Theta
        K = np.argmin(delta_angle, axis=2) 
        def plot(mtrx):
            """ Used for debugging """
            self.inflated_map = np.ceil((mtrx.T - np.min(mtrx)) / (np.max(mtrx) - np.min(mtrx)) * 100)
            
        relevant_delta_angles = np.min(delta_angle, axis=2)
        relevant_measurements = np.minimum(np.take(measurements,K, axis=0), self.z_max+1) 
        shifted_relevant_measurements = relevant_measurements + self.alpha / 2 * np.ones((self.map_height, self.map_width))
        b1 = R_map > np.minimum(self.z_max * np.ones((self.map_height, self.map_width)), shifted_relevant_measurements) 
        b2 = relevant_delta_angles > self.beta / 2 
       
        bool_mtrx_1 = b1 | b2

        b3 = relevant_measurements < self.z_max
        b4 = np.abs(R_map - relevant_measurements) < self.alpha / 2

        bool_mtrx_2 = b3 & b4
        bool_mtrx_3 = R_map < relevant_measurements
        
        bool_mtrx_1 = bool_mtrx_1.astype(int)
        bool_mtrx_2 = bool_mtrx_2.astype(int)
        bool_mtrx_3 = bool_mtrx_3.astype(int)

        delta_log_odds = (self.L_0 * bool_mtrx_1 + self.L_OCC * bool_mtrx_2 + self.L_FREE * bool_mtrx_3).T - self.L_0 * np.ones((self.map_width, self.map_height)) 

        delta_log_odds[(R_map > self.z_max).T] = 0
        delta_log_odds[(R_map > shifted_relevant_measurements).T] = 0
        delta_log_odds[(R_map < 0.05).T] = 0
        return delta_log_odds
    
    def inverse_sensor_model(self, measurements: np.ndarray, x: np.ndarray, downsampled=False ):
        """ Assuming x = [x,y,theta], with theta in radians """

        if downsampled: 
            resolution = self.downsampled_resolution
            origin = self.downsampled_origin
            width = self.downsampled_width
            height = self.downsampled_height
        else: 
            resolution = self.resolution
            origin = self.origin
            width = self.map_width
            height = self.map_height

        start_time = time.time()
        measurements = self.median_filter_1d(measurements, 9)

        # Get the index of the cell in the map  
        idx = self.from_pos_to_idx(x, resolution, origin)
        idx_x, idx_y = idx[0], idx[1]

        # Get the computation slice from the max range 
        index_range = self.from_pos_to_idx(np.array([self.z_max, self.z_max, 0]), resolution, origin)

        # Calculate bounds once
        start_idx_x = np.maximum(0, idx_x - index_range[0])
        end_idx_x = np.minimum(width, idx_x + index_range[0])
        start_idx_y = np.maximum(0, idx_y - index_range[1])
        end_idx_y = np.minimum(height, idx_y + index_range[1]) 

        # Calculate lengths once
        len_x = end_idx_x - start_idx_x 
        len_y = end_idx_y - start_idx_y 
        
        # Pre-allocate arrays and use in-place operations where possible
        x_range = np.arange(start_idx_x, end_idx_x) + 0.5
        y_range = np.arange(start_idx_y, end_idx_y) + 0.5
        
        # More efficient array creation
        x_dist = (np.tile(x_range, (len_y, 1)) - idx_y) * resolution
        y_dist = (np.tile(y_range, (len_x, 1)).T - idx_x) * resolution

        # Distance from the robot to the cell
        R_map = np.hypot(x_dist, y_dist) 
        
        # relative angle between the robot and the cell
        phi = np.arctan2(x_dist, y_dist) - x[2] 
        #  phi = np.mod(phi + np.pi, 2*np.pi) - np.pi  

        # Pre-calculate measurement angles
        measurement_angles = (np.arange(len(measurements)) * (np.pi / 180) - np.pi)
        
        # More efficient 3D array operations
        Phi = phi[..., np.newaxis]
        Theta = measurement_angles[np.newaxis, np.newaxis, :]
        
        # More efficient angle difference calculation
        delta_angle = np.angle(np.exp(1j * (Phi - Theta + np.pi))) - np.pi
        del Phi, Theta  # Free memory early

        # Get relevant indices and measurements
        K = np.argmin(delta_angle, axis=2)
        relevant_delta_angles = np.min(delta_angle, axis=2)
        del delta_angle  # Free memory
        
        # More efficient measurement processing
        relevant_measurements = np.minimum(measurements[K], self.z_max + 1) - self.alpha
        shifted_relevant_measurements = relevant_measurements + self.alpha / 2
        
        # Calculate boolean matrices more efficiently
        z_max_array = self.z_max  # Scalar comparison instead of creating array
        
        bool_mtrx_1 = ((R_map > np.minimum(z_max_array, shifted_relevant_measurements)) | (relevant_delta_angles > self.beta / 2)).astype(np.int8)
        bool_mtrx_2 = (relevant_measurements < z_max_array) & (np.abs(R_map - relevant_measurements) <= self.alpha / 2).astype(np.int8)
        bool_mtrx_3 = (R_map < relevant_measurements).astype(np.int8)

        # Calculate delta_log_odds more efficiently
        delta_log_odds = np.zeros((len_x, len_y), dtype=np.float32)
        delta_log_odds += self.L_0 * bool_mtrx_1.T
        delta_log_odds += self.L_OCC * bool_mtrx_2.T
        delta_log_odds += self.L_FREE * bool_mtrx_3.T
        delta_log_odds -= self.L_0
        
        # Apply masks more efficiently
        mask = (R_map > z_max_array) | (R_map > shifted_relevant_measurements)
        delta_log_odds[mask.T] = 0
        
        # Create final output more efficiently
        final_delta_log_odds = np.zeros((width, height), dtype=np.float32)
        final_delta_log_odds[start_idx_x:end_idx_x, start_idx_y:end_idx_y] = delta_log_odds
        end_time = time.time()
        print(f"Time taken: {end_time - start_time} seconds")
        return final_delta_log_odds

    def inflate_map(self, map: np.ndarray, kernel_size: int, cutoff: int):
        """ Inflate the map by 1 cell """
        thresholded_map = np.array(map > 50, dtype=np.float32) * 100
        blurred_map = cv2.GaussianBlur(thresholded_map, (kernel_size,kernel_size), 0)
        return np.array(blurred_map > cutoff, dtype=np.int8) * 100

    def recover_probabilities(self, log_odds: np.ndarray):
        """ Recover probabilities from log odds """
        return ( 100 * (1 - 1 / (1 + np.exp(log_odds)) ) ).astype(int)
    
    def compute_inflated_occupancy(self):
        """ Compute the inflated occupancy map """
        log_odds = self.log_odds if not self.should_downsample else self.upsample_map(self.log_odds)
        self.map = self.recover_probabilities(log_odds)
        # self.inflated_map = self.map
        self.map_binary = np.array(self.map >=99, dtype=np.int8) * 100
        self.inflated_map = self.inflate_map(self.map, 27, 8)
        self.inflated_map_planning = self.inflate_map(self.map, 29, 3)

    def upsample_map(self, map: np.ndarray):
        """ Upsample the map to the original resolution """
        scale_factor = int(self.downsampled_resolution)
        
        upsampled = cv2.resize(
            map, 
            (self.map_height, self.map_width), 
            interpolation=cv2.INTER_NEAREST
        )
        return upsampled.astype(np.int8)

    def update_map(self, measurements: np.ndarray, x: np.ndarray):
        """ Update the map with the new measurements """
        self.log_odds = np.clip(self.log_odds + self.inverse_sensor_model(measurements, x, downsampled=self.should_downsample), -10, 30)

