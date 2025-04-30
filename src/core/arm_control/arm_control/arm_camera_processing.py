import numpy as np
import cv2
from scipy.spatial import KDTree

class ArmCameraProcessing:

    def __init__(self):
        self.fx = 540.0
        self.fy = 540.0
        self.cx = 320.0
        self.cy = 240.0
        
    def convert_image_to_world_coordinates(self, x, y, Z):
        # Convert the point to world coordinates
        X = (x - self.cx) * Z / self.fx
        Y = (y - self.cy) * Z / self.fy
        return X, Y, Z

    def get_frame(self):
        ret, frame = self.camera.read()
        return frame

    def morphological_opening(self, frame):
        kernel = np.ones((30,30), np.uint8)
        return cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    
    def morphological_closing(self, frame):
        kernel = np.ones((30,30), np.uint8)
        return cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
    
    def morphological_erosion(self, frame):
        kernel = np.ones((30,30), np.uint8)
        return cv2.erode(frame, kernel, iterations=1) 

    def morphological_dilation(self, frame):
        kernel = np.ones((30,30), np.uint8)
        return cv2.dilate(frame, kernel, iterations=1)

    def check_gripper_reach(self, point1, point2, gripper_reach_max=200):
        dist = np.linalg.norm(point1 - point2)
        if dist < gripper_reach_max:
            return True
        else:
            print(f"Distance: {dist}")
            return False
        
    def compute_primary_axis(self, point, edge_points, neighbor_radius=5):
        """Compute the tangent direction at a point using multiple methods.
        
        Args:
            point: The point at which to compute the tangent
            edge_points: All edge points in the component
            neighbor_radius: Radius for considering neighboring points
            
        Returns:
            tuple: (primary_axis, flatness)
        """
        # Get neighborhood of point using KD-tree for efficiency
        kdtree = KDTree(edge_points)
        neighbor_indices = kdtree.query_ball_point(point, neighbor_radius)
        if len(neighbor_indices) < 3:  # Need at least 3 points for robust estimation
            return np.array([1, 0]), 1.0
            
        neighborhood = edge_points[neighbor_indices]
        
        # Method 1: PCA-based direction estimation
        # Center the neighborhood
        centered = neighborhood - np.mean(neighborhood, axis=0)
        # Compute covariance matrix
        covariance_matrix = np.cov(centered, rowvar=False)
        # Get eigenvectors and eigenvalues
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        # Sort in descending order
        sorted_indices = np.argsort(eigenvalues)[::-1]
        pca_direction = eigenvectors[:, sorted_indices[0]]
        
        # Method 2: Local polynomial fitting
        # Sort points by distance along PCA direction
        proj = np.dot(centered, pca_direction)
        sort_idx = np.argsort(proj)
        sorted_points = centered[sort_idx]
        
        # Fit second-degree polynomial
        try:
            coeffs = np.polyfit(sorted_points[:, 0], sorted_points[:, 1], 2)
            # Compute derivative at center point
            poly_direction = np.array([1.0, 2 * coeffs[0] * 0 + coeffs[1]])
            poly_direction /= np.linalg.norm(poly_direction)
        except:
            poly_direction = pca_direction
            
        # Method 3: Finite differences using closest points
        closest_indices = kdtree.query(point, k=3)[1][1:]  # Get 2 closest points excluding self
        if len(closest_indices) >= 2:
            p1, p2 = edge_points[closest_indices]
            fd_direction = p2 - p1
            fd_direction = fd_direction / np.linalg.norm(fd_direction)
        else:
            fd_direction = pca_direction
            
        # Combine the estimates
        # Ensure all vectors point in similar direction
        if np.dot(poly_direction, pca_direction) < 0:
            poly_direction = -poly_direction
        if np.dot(fd_direction, pca_direction) < 0:
            fd_direction = -fd_direction
            
        # Weight the estimates based on confidence
        weights = np.array([
            1.0,  # PCA weight
            0.8 if len(neighborhood) > 5 else 0.0,  # Polynomial fit weight
            0.6  # Finite difference weight
        ])
        weights = weights / np.sum(weights)
        
        combined_direction = (
            weights[0] * pca_direction + 
            weights[1] * poly_direction + 
            weights[2] * fd_direction
        )
        combined_direction = combined_direction / np.linalg.norm(combined_direction)
        
        # Compute measure of flatness using ratio of eigenvalues
        if eigenvalues[sorted_indices[0]] > 0:
            flatness = eigenvalues[sorted_indices[1]] / eigenvalues[sorted_indices[0]]
        else:
            flatness = 1.0
            
        # Additional confidence measure based on consistency of estimates
        consistency = np.mean([
            np.abs(np.dot(pca_direction, poly_direction)),
            np.abs(np.dot(pca_direction, fd_direction)),
            np.abs(np.dot(poly_direction, fd_direction))
        ])
        
        # Adjust flatness based on consistency
        flatness = flatness * consistency
        
        return combined_direction, flatness
        
    def check_colinearity(self, primary_axis1, primary_axis2, threshold = 0.8):
        # Compute the dot product of the two primary axes
        normalized_primary_axis1 = primary_axis1 / np.linalg.norm(primary_axis1) 
        normalized_primary_axis2 = primary_axis2 / np.linalg.norm(primary_axis2)
        dot_product = np.abs(np.dot(normalized_primary_axis1, normalized_primary_axis2))

        if dot_product > threshold:
            return True
        else:
            return False
        
    def check_surface_gripper_orthogonality(self, point1, point2, primary_axis1, primary_axis2, threshold = 0.1):
        # Compute the normalized vector between the two points 
        vector1 = point2 - point1
        normalized_vector1 = vector1 / np.linalg.norm(vector1)

        # Compute the dot product of the normalized vector and each primary axis
        dot_product1 = np.dot(normalized_vector1, primary_axis1)
        dot_product2 = np.dot(normalized_vector1, primary_axis2)

        if max(abs(dot_product1), abs(dot_product2)) < threshold:
            return True
        else:
            return False
        
    def check_grip_balance(self, point1, point2, edge_points, threshold=0.4):
        """Check if the grip divides the contour into two roughly equal parts.
        
        Args:
            point1, point2: The two gripping points
            edge_points: All points in the contour
            threshold: Minimum ratio between smaller and larger part (0 to 1)
            
        Returns:
            bool: True if the grip creates a balanced division
        """
        # Compute grip line vector and its normal
        grip_vector = point2 - point1
        grip_center = (point1 + point2) / 2
        grip_length = np.linalg.norm(grip_vector)
        normalized_grip = grip_vector / grip_length
        
        # Normal vector to the grip line
        normal_vector = np.array([-normalized_grip[1], normalized_grip[0]])
        
        # Project all points onto the normal vector relative to grip center
        projections = np.dot(edge_points - grip_center, normal_vector)
        
        # Count points on each side of the grip line
        positive_side = np.sum(projections > 0)
        negative_side = np.sum(projections < 0)
        
        # Calculate balance ratio
        balance_ratio = min(positive_side, negative_side) / max(positive_side, negative_side)
        
        # Check local balance near grip points
        local_radius = grip_length * 0.5  # Use grip length to determine local neighborhood
        
        # Count points in local neighborhoods
        local_points1 = edge_points[np.linalg.norm(edge_points - point1, axis=1) < local_radius]
        local_points2 = edge_points[np.linalg.norm(edge_points - point2, axis=1) < local_radius]
        
        # Check local balance for each grip point
        local_proj1 = np.dot(local_points1 - point1, normal_vector)
        local_proj2 = np.dot(local_points2 - point2, normal_vector)
        
        local_balance1 = min(np.sum(local_proj1 > 0), np.sum(local_proj1 < 0)) / max(1, max(np.sum(local_proj1 > 0), np.sum(local_proj1 < 0)))
        local_balance2 = min(np.sum(local_proj2 > 0), np.sum(local_proj2 < 0)) / max(1, max(np.sum(local_proj2 > 0), np.sum(local_proj2 < 0)))
        
        # Combined balance score
        local_balance = min(local_balance1, local_balance2)
        
        # Check both global and local balance
        is_balanced = (balance_ratio > threshold and local_balance > threshold * 0.8)
        
        if not is_balanced:
            print(f"Balance ratio: {balance_ratio:.2f}, Local balance: {local_balance:.2f}")
            
        return is_balanced
        
    def check_grip_validity(self, point1, point2, edge_points, frame):
        # Check reachability 
        if not self.check_gripper_reach(point1, point2):
            return False
        
        # Check colinearity 
        primary_axis1, flatness1 = self.compute_primary_axis(point1, edge_points)
        primary_axis2, flatness2 = self.compute_primary_axis(point2, edge_points)

        # Check flatness 
        if min(flatness1, flatness2) > 0.1:
            return False
        # Check colinearity 
        if not self.check_colinearity(primary_axis1, primary_axis2):
            return False
        # Check surface gripper orthogonality 
        if not self.check_surface_gripper_orthogonality(point1, point2, primary_axis1, primary_axis2):
            return False
        # Check grip balance 
        if not self.check_grip_balance(point1, point2, edge_points):
            return False
        return True

    def grip_rejection_sampling(self, frame, edge_points, MAX_ITERATIONS=1000, THRESHOLD=10): 
        """Sample valid gripping points from edge points.
        
        Args:
            edge_points: Nx2 array of edge point coordinates
            MAX_ITERATIONS: Maximum number of sampling attempts
            THRESHOLD: Distance threshold for grip validation
            
        Returns:
            tuple: (success, points), where success is bool and points is array of two points
        """
        for i in range(MAX_ITERATIONS):
            # Sample two random indices
            idx = np.random.choice(len(edge_points), size=2, replace=False)
            # Get the corresponding points
            sample_points = edge_points[idx]

            # Check validity of the grip 
            if self.check_grip_validity(sample_points[0], sample_points[1], edge_points, frame):
                return True, sample_points
        
        return False, None
            
    def best_edge_detection(self, frame):
        red = frame[:, :, 2]
        green = frame[:, :, 1]
        blue = frame[:, :, 0] 

        red_blurred_lower = cv2.GaussianBlur(red, (21, 21), 50)
        green_blurred_lower = cv2.GaussianBlur(green, (21, 21), 50)
        blue_blurred_lower = cv2.GaussianBlur(blue, (21, 21), 50)

        red_blurred_upper = cv2.GaussianBlur(red, (15, 15), 50)
        green_blurred_upper = cv2.GaussianBlur(green, (15, 15), 50)
        blue_blurred_upper = cv2.GaussianBlur(blue, (15, 15), 50)

        # Threshold each channel separately Keep everyting BELOW the trheshold 
        _, red_thresh_lower = cv2.threshold(red_blurred_lower, 50, 255, cv2.THRESH_BINARY)
        _, green_thresh_lower = cv2.threshold(green_blurred_lower, 50, 255, cv2.THRESH_BINARY)
        _, blue_thresh_lower = cv2.threshold(blue_blurred_lower, 50, 255, cv2.THRESH_BINARY)

        _, red_thresh_upper = cv2.threshold(red_blurred_upper, 180, 255, cv2.THRESH_BINARY_INV)
        _, green_thresh_upper = cv2.threshold(green_blurred_upper, 180, 255, cv2.THRESH_BINARY_INV)
        _, blue_thresh_upper = cv2.threshold(blue_blurred_upper, 180, 255, cv2.THRESH_BINARY_INV)

        # Combine the information stored in the lower and upper thresholded images
        bright_pixels = red_thresh_lower & green_thresh_lower & blue_thresh_lower
        dark_pixels = red_thresh_upper & green_thresh_upper & blue_thresh_upper
        notable_pixels = bright_pixels & dark_pixels

        # Apply morphological operations
        notable_erosion = self.morphological_erosion(notable_pixels) 
        notable_erosion_dilation = self.morphological_dilation(notable_erosion)

        edges_notable_erosion_dilation = cv2.Canny(notable_erosion_dilation, 50, 150)

        return edges_notable_erosion_dilation

    def get_connected_components(self, edge_points, distance_threshold=10):
        """Find connected edge segments using BFS.
        
        Args:
            edge_points: Nx2 array of edge point coordinates
            distance_threshold: Maximum distance between connected points
        """
        kdtree = KDTree(edge_points)
        connected_components = []
        visited = set()

        def bfs_component(start_idx):
            component = []
            queue = [start_idx]
            
            while queue:
                current_idx = queue.pop(0)
                if current_idx in visited:
                    continue
                    
                visited.add(current_idx)
                component.append(current_idx)
                
                # Find neighbors within distance threshold
                neighbors = kdtree.query_ball_point(edge_points[current_idx], distance_threshold)
                
                # Add unvisited neighbors to queue
                queue.extend([n for n in neighbors if n not in visited])
                
            return component

        # Find components using BFS
        for i in range(len(edge_points)):
            if i not in visited:
                component = bfs_component(i)
                if len(component) > 10:  # Filter out tiny components
                    connected_components.append(component)

        return connected_components

    def filter_connected_component_by_size(self, connected_components, edge_points, 
                                         min_points=10, min_area=100, min_perimeter=20):
        """Filter components based on multiple size metrics.
        
        Args:
            connected_components: List of component indices
            edge_points: Complete set of edge points
            min_points: Minimum number of points in component
            min_area: Minimum area enclosed by component
            min_perimeter: Minimum perimeter length
        """
        filtered_components = []
        
        for component in connected_components:
            # Get actual points for this component
            points = edge_points[component]
            
            # 1. Check number of points
            if len(points) < min_points:
                continue
                
            # 2. Calculate and check area
            # Get bounding box
            min_coords = np.min(points, axis=0)
            max_coords = np.max(points, axis=0)
            width = max_coords[1] - min_coords[1]
            height = max_coords[0] - min_coords[0]
            
            # Create binary mask for component
            mask = np.zeros((int(height) + 1, int(width) + 1), dtype=np.uint8)
            local_points = points - [min_coords[0], min_coords[1]]
            mask[local_points[:, 0], local_points[:, 1]] = 255
            
            # Fill the contour
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                cv2.drawContours(mask, contours, -1, 255, -1)  # -1 means fill
                area = np.sum(mask > 0)
                
                if area < min_area:
                    continue
                
                # 3. Calculate and check perimeter
                perimeter = cv2.arcLength(contours[0], True)
                if perimeter < min_perimeter:
                    continue
                
                # 4. Calculate compactness (circularity)
                compactness = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                
                # 5. Calculate aspect ratio
                aspect_ratio = width / height if height > 0 else 0

                filtered_components.append(component)
        
        return filtered_components

    def filter_connected_components_by_closedness(self, connected_components, edge_points, closedness_threshold=0.9):
        # Filter out connected components that are not closed
        filtered_components = []
        for component in connected_components:
            if self.is_closed(component, edge_points):
                filtered_components.append(component)
        return filtered_components
    
    def is_closed(self, component_indices, edge_points, max_gap=5, min_closedness=0.85, gap_tolerance=3):
        """Check if a component forms a closed contour, allowing for small gaps.
        
        Args:
            component_indices: Indices of points in the component
            edge_points: Complete set of edge points
            max_gap: Maximum allowed gap between points in the contour
            min_closedness: Minimum ratio of connected points required
            gap_tolerance: Number of pixels to dilate to bridge small gaps
        
        Returns:
            bool: True if the component forms a closed contour
        """
        points = edge_points[component_indices]
        
        # 1. Create a binary mask of the component with padding
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        height = int(max_coords[0] - min_coords[0] + 1)
        width = int(max_coords[1] - min_coords[1] + 1)
        
        if height < 3 or width < 3:  # Too small to be closed
            return False
            
        # Create mask with extra padding for dilation
        pad = gap_tolerance + 2
        mask = np.zeros((height + 2*pad, width + 2*pad), dtype=np.uint8)
        local_points = points - min_coords
        mask[local_points[:, 0] + pad, local_points[:, 1] + pad] = 255
        
        # Dilate to bridge small gaps
        kernel = np.ones((gap_tolerance, gap_tolerance), np.uint8)
        dilated_mask = cv2.dilate(mask, kernel, iterations=1)
        
        # 2. Check if the dilated component forms a complete boundary
        flood_mask = dilated_mask.copy()
        cv2.floodFill(flood_mask, None, (0, 0), 128)
        
        # Check for enclosed areas (unreached pixels)
        unreached = np.sum(flood_mask == 0)
        if unreached == 0:  # No enclosed area
            return False
            
        # 3. Check connectivity allowing for gaps
        kdtree = KDTree(points)
        well_connected = 0
        
        for point in points:
            # Find nearest neighbors within max_gap + gap_tolerance
            neighbors = kdtree.query_ball_point(point, max_gap + gap_tolerance)
            if len(neighbors) >= 3:  # Need at least 2 neighbors besides self
                well_connected += 1
        
        connectivity_ratio = well_connected / len(points)
        
        # 4. Check contour properties on the dilated mask
        contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 1:  # Should be exactly one contour
            return False
            
        # Calculate contour metrics
        perimeter = cv2.arcLength(contours[0], True)
        area = cv2.contourArea(contours[0])
        
        # Compute circularity with relaxed threshold for gapped contours
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        
        # Adjust points per perimeter threshold based on gap tolerance
        min_points_per_perimeter = 0.25 / (1 + gap_tolerance * 0.1)  # Decrease threshold as gap tolerance increases
        points_per_perimeter = len(points) / perimeter if perimeter > 0 else 0
        
        return (connectivity_ratio >= min_closedness and 
                circularity > 0.05 and  # More relaxed circularity threshold
                points_per_perimeter >= min_points_per_perimeter)

    def filter_connected_components(self, connected_components, edge_points, min_size=100):
        # Filter by size 
        large_components = self.filter_connected_component_by_size(connected_components, edge_points, min_points=min_size, min_perimeter=200)
        # Filter by closedness 
        closed_components = self.filter_connected_components_by_closedness(large_components, edge_points)
        
        return closed_components
    
    def get_gripping_position(self, frame):
        edges = self.best_edge_detection(frame)
        edge_points = np.where(edges == 255) 
        edge_points = np.array(edge_points).T
        connected_components = self.get_connected_components(edge_points)
        connected_components = self.filter_connected_components(connected_components, edge_points)
        gripping_positions = []
        components_to_remove = []
        for component in connected_components: 
            success, points = self.grip_rejection_sampling(frame, edge_points[component]) 
            if not success:  
                components_to_remove.append(component) 
                continue
            gripping_positions.append(points)
        for component in components_to_remove:
            connected_components.remove(component)
        return gripping_positions, connected_components
    
class BoxDetection:
    def __init__(self):
        pass
        # self.camera = cv2.VideoCapture(0)
        # self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def get_frame(self):
        ret, frame = self.camera.read()
        return frame
    
    def get_edges(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blurred = cv2.GaussianBlur(gray, (51, 51), 10)

        thres = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 111, 25)

        edges = cv2.Canny(thres, 50, 150)
        return edges
    
    def get_contours(self, edges):
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours
    
    def RANSAC_HoughLines(self, edges):
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)
        return lines
    
    def sort_lines_by_length(self, lines):
        """Sort lines by length in descending order."""
        # First, ensure all lines have the expected format
        valid_lines = []
        for line in lines:
            try:
                # Check if line has the expected format [x1, y1, x2, y2]
                if len(line[0]) == 4:
                    valid_lines.append(line)
                else:
                    print(f"Skipping line with unexpected format: {line}")
            except (IndexError, TypeError) as e:
                print(f"Skipping invalid line: {line}, Error: {e}")
        
        # Calculate length using Euclidean distance
        def line_length(line):
            x1, y1, x2, y2 = line[0]
            return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Sort by length in descending order
        sorted_lines = sorted(valid_lines, key=line_length, reverse=True)

        longest_lines = [sorted_lines[0]]
        if len(sorted_lines) < 2: 
            return []
        
        for line in sorted_lines[1:]: 
            if not self.is_collinear(longest_lines[0], line):
                longest_lines.append(line)
                break
                
        return longest_lines
        
    def is_collinear(self, line1, line2):
        x1, y1, x2, y2 = line1[0]
        x3, y3, x4, y4 = line2[0]
        # Calculate the angle between the two lines
        angle = np.arctan2(y4 - y3, x4 - x3) - np.arctan2(y2 - y1, x2 - x1)
        return np.abs(angle) < np.pi / 4
    
    def get_closest_endpoints(self, lines):
        # Get the two longest lines
        # longest_lines = self.sort_lines_by_length(lines)
        longest_lines = lines

       # Find the pair of endpoints that are the closest to each other
        min_distance = float('inf')
        print("using:", longest_lines[0])
        x1, y1, x2, y2 = longest_lines[0][0]
        x3, y3, x4, y4 = longest_lines[1][0]
        endpoints1 = [np.array([x1, y1, x2, y2]), np.array([x2, y2, x1, y1])]
        endpoints2 = [np.array([x3, y3, x4, y4]), np.array([x4, y4, x3, y3])]
        min_pair = None
        for i in range(len(endpoints1)): 
            for j in range(len(endpoints2)):
                distance = np.sqrt((endpoints1[i][0] - endpoints2[j][0])**2 + (endpoints1[i][1] - endpoints2[j][1])**2)
                if distance < min_distance:
                    min_distance = distance
                    min_pair = [endpoints1[i], endpoints2[j]]

        return min_pair 
    
    def get_intersection(self, line1, line2):
        """Calculate the intersection point of two lines.
        
        Args:
            line1: First line in format [[x1, y1, x2, y2]]
            line2: Second line in format [[x3, y3, x4, y4]]
        """
        # Extract points
        x1, y1, x2, y2 = map(float, line1)
        x3, y3, x4, y4 = map(float, line2)
        
        # Calculate the determinant
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-8:
            return None

        # Calculate intersection point
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
        
        return (int(round(px)), int(round(py)))
    
    def get_parallelogram_corners(self, vectors):
        vec1 = vectors[0]
        vec2 = vectors[1]
        v1 = vec1[2:] - vec1[:2]
        v2 = vec2[2:] - vec2[:2]
        pts = np.array([
            vec1[:2], 
            vec1[:2] + v1,
            vec1[:2] + v1 + v2, 
            vec2[:2] + v2, 
            vec2[:2]
        ]) 

        pts = pts.reshape((-1, 1, 2))
        return pts

    def draw_lines(self, frame, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return frame
    
    def inside_place_region(self, new_point, vec1, vec2):
        v1 = vec1[2:] - vec1[:2]
        v2 = vec2[2:] - vec2[:2]

        # Find the vector oriented to the right of the other 
        right = None 
        left = None
        cross_product = self.cross_2d(v1, v2)
        if cross_product > 0:
            right = v1
            left = v2
        else:
            right = v2
            left = v1
        new_vec_rel_right = new_point - right[:2]
        new_vec_rel_left = new_point - left[:2]
        # A new point is inside the region if it is on the left side of the right one and the right side of the left one
        new_cross_1 = self.cross_2d(right, new_vec_rel_right) 
        new_cross_2 = self.cross_2d(left, new_vec_rel_left)
        return new_cross_1 > 0 and new_cross_2 < 0
    
    def draw_region_for_placing(self, frame, endpoints): 
        # Draw with green color and light opacity so that it is see through 
        dimensions = frame.shape
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                if self.inside_place_region(np.array([i, j]), endpoints[0], endpoints[1]):
                    print(i, j)
                    # Draw on top of the original image
                    frame[i, j] = (0, 255, 0)
        return frame
        
    def cross_2d(self, vec1, vec2):
        return vec1[0] * vec2[1] - vec1[1] * vec2[0]
    
    def draw_place_region(self, frame, endpoints):
        # Draw a circle at each endpoint
        for e in endpoints:
            cv2.circle(frame, (e[0], e[1]), 5, (0, 0, 255), -1)
        return frame
    
    def get_contour_from_image(self, frame):
        box_detection = BoxDetection()
        edges = box_detection.get_edges(frame)
        contours = box_detection.get_contours(edges)

        # Create a blank image to draw contours on
        contour_image = np.zeros_like(frame)  # or use np.zeros(frame.shape, dtype=np.uint8)

        # Draw the contours on the blank image
        cv2.drawContours(contour_image, contours, -1, (255, 255, 255), 2)  # -1 means draw all contours
        
        lines = box_detection.RANSAC_HoughLines(edges)

        lines = box_detection.sort_lines_by_length(lines)
        frame = box_detection.draw_lines(frame, lines)

        endpoints = box_detection.get_closest_endpoints(lines)
        intersection = box_detection.get_intersection(endpoints[0], endpoints[1])
        endpoints[0][:2] = intersection
        endpoints[1][:2] = intersection
        pts = box_detection.get_parallelogram_corners(endpoints)
        sum = np.array([0, 0])
        for p in pts:
            sum[0] += p[0][0]
            sum[1] += p[0][1]
        center_point = sum / len(pts)
        color = (0, 255, 0)
        # frame = cv2.fillPoly(frame, [pts], color)
        # edges_as_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return center_point

class BoxPositionConverter:
    def __init__(self):
        self.fx = 540.0
        self.fy = 540.0
        self.cx = 305.0
        self.cy = 243.0

    def convert_image_to_world_coordinates(self, transform_arm_base_link_to_arm_camera, x, y, plane_z_coordinate, logger = None):
        # Compute the unit vector for the point in the image 

        u = (x - self.cx) / self.fx
        v = (y - self.cy) / self.fy

        R = transform_arm_base_link_to_arm_camera[:3, :3]
        t = transform_arm_base_link_to_arm_camera[:3, 3] 

        t_z = t[2] 
        alpha = - (np.arccos(R[2,2]) - np.pi / 2)
        phi = np.arctan2(np.sqrt(u**2 + v**2), 1) * np.sign(v)
        beta = alpha - phi
        angle_scaling = 1 / np.sin(beta) * np.cos(phi)
        # if logger is not None:
        #     logger.info(f"R[2,2]: {R[2,2]}, arccos(R[2,2]) - pi/2: {alpha}")
        #     logger.info(f"phi: {phi}")
        #     logger.info(f"u: {u}, v: {v}")
        scaling = (plane_z_coordinate - t_z ) * angle_scaling
        X = u * scaling     
        Y = v * scaling 
        Z = scaling 

        # Convert to arm_base_link frame 
        pos = np.array([X, Y, Z, 1])
        pos = transform_arm_base_link_to_arm_camera @ pos
        X, Y, Z = pos[:3]

        return X, Y, Z