import numpy as np
import matplotlib.pyplot as plt

class Polygon:
    def __init__(self, verticies: list[np.ndarray]):
        self.verticies = verticies

    def __str__(self):
        return f"Polygon with verticies {self.verticies}"

    def __repr__(self):
        return f"Polygon({self.verticies})"
    
    def __len__(self):
        return len(self.verticies)
    
    def _cow(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> bool:  
        # Counter-clockwise orientation test
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def _on_segment(self, p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
        """Check if point q lies on segment pr"""
        return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

    def _orientation(self, p: np.ndarray, q: np.ndarray, r: np.ndarray) -> int:
        """Returns orientation of ordered triplet (p, q, r).
        Returns:
         0 --> Collinear
         1 --> Clockwise
         2 --> Counterclockwise"""
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        return 1 if val > 0 else 2

    def line_segment_intersect(self, p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> bool:
        """Check if line segments p1q1 and p2q2 intersect"""
        # Find orientations
        o1 = self._orientation(p1, q1, p2)
        o2 = self._orientation(p1, q1, q2)
        o3 = self._orientation(p2, q2, p1)
        o4 = self._orientation(p2, q2, q1)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special Cases: collinear segments
        if o1 == 0 and self._on_segment(p1, p2, q1): return True
        if o2 == 0 and self._on_segment(p1, q2, q1): return True
        if o3 == 0 and self._on_segment(p2, p1, q2): return True
        if o4 == 0 and self._on_segment(p2, q1, q2): return True

        return False
    
    def segment_intersects_polygon(self, start:np.ndarray, end:np.ndarray) -> bool:
        """
        Tests if a line segment intersects with any edge of the polygon.
        
        Args:
            start: Starting point of the line segment
            end: Ending point of the line segment
            
        Returns:
            True if the segment intersects with any polygon edge, False otherwise
        """
        for i in range(len(self.verticies)):
            edge_start = self.verticies[i] 
            edge_end = self.verticies[(i + 1) % len(self.verticies)]
            if self.line_segment_intersect(start, end, edge_start, edge_end):
                return True 
        return False
    
    def is_internal(self, point: np.ndarray) -> bool:
        # Count number of times a ray from the point to infinity crosses the polygon 
        count = 0
        for i in range(len(self.verticies)):
            edge_start = self.verticies[i] 
            edge_end = self.verticies[(i + 1) % len(self.verticies)]
            # Skip if point's y is not within edge's y range
            if not min(edge_start[1], edge_end[1]) < point[1] <= max(edge_start[1], edge_end[1]):
                continue 
            # Check if the ray crosses the edge 
            if edge_start[0] == edge_end[0]:  # Vertical edge
                if point[0] <= edge_start[0]:
                    count += 1
            else:
                intersection_x = (point[1] - edge_start[1]) * (edge_end[0] - edge_start[0]) / (edge_end[1] - edge_start[1]) + edge_start[0]
                if point[0] <= intersection_x:
                    count += 1
        return count % 2 == 1
    
    def rejection_sample(self, n_samples: int, bounds: tuple[tuple[float, float], tuple[float, float]] = None) -> np.ndarray:
        """
        Perform rejection sampling to generate points inside the polygon.
        
        Args:
            n_samples: Number of samples to generate
            bounds: ((x_min, x_max), (y_min, y_max)) for sampling. If None, uses polygon bounds.
        
        Returns:
            Array of shape (n_samples, 2) containing points inside the polygon
        """
        if bounds is None:
            # Calculate bounds from polygon vertices
            vertices = np.array(self.verticies)
            x_min, y_min = vertices.min(axis=0)
            x_max, y_max = vertices.max(axis=0)
        else:
            (x_min, x_max), (y_min, y_max) = bounds
        
        samples = []
        while len(samples) < n_samples:
            # Generate random point within bounds
            point = np.array([
                np.random.uniform(x_min, x_max),
                np.random.uniform(y_min, y_max)
            ])
            if self.is_internal(point):
                samples.append(point)
        
        return np.array(samples)

if __name__ == "__main__":
    # Test
    vertices = [ 
        np.array([0, 0]),
        np.array([1, 0]), 
        np.array([1, 0.8]),
        np.array([0.5, 0.5]),
        np.array([0.8, 1]),
        np.array([0, 1])
    ]
    polygon = Polygon(vertices)

    while True:
        print(polygon)
        start_x = float(input("Enter x1 coordinate: "))
        start_y = float(input("Enter y1 coordinate: "))
        end_x = float(input("Enter x2 coordinate: "))
        end_y = float(input("Enter y2 coordinate: "))
        print(polygon.segment_intersects_polygon(np.array([start_x, start_y]), np.array([end_x, end_y])))
