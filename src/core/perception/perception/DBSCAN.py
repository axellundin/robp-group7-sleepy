import numpy as np
from collections import Counter

class DBSCAN:
    def __init__(self, eps=1, min_samples=4):
        self.eps = eps
        self.min_samples = min_samples
        self.labels = None
    
    def fit_predict(self, points):
        n_points = len(points)
        self.labels = np.full(n_points, -1)  
        cluster_id = 0
        for i in range(n_points):
            if self.labels[i] != -1: 
                continue
            neighbors = self.get_neighbors(i, points)
            if len(neighbors) < self.min_samples:  
                self.labels[i] = -1
            else:
                self._expand_cluster(i, neighbors, cluster_id, points)
                cluster_id += 1

        return self.labels

    def get_neighbors(self, point_idx, points):
        neighbors = []
        for i, point in enumerate(points):
            if np.linalg.norm(points[point_idx] - point) <= self.eps:
                neighbors.append(i)
        return neighbors

    def _expand_cluster(self, point_idx, neighbors, cluster_id, points):
        self.labels[point_idx] = cluster_id
        to_visit = neighbors.copy() 
        visited_points = set([point_idx])  
        
        while to_visit:
            current_point_idx = to_visit.pop()
            if self.labels[current_point_idx] == -1:  
                self.labels[current_point_idx] = cluster_id
            
            current_neighbors = self.get_neighbors(current_point_idx, points)
            for neighbor in current_neighbors:
                if self.labels[neighbor] == -1 and neighbor not in visited_points:
                    to_visit.append(neighbor)
                    visited_points.add(neighbor) 

    def get_labels(self):
        return self.labels
    
class better_DBSCAN:
    def __init__(self, eps=1, min_samples=4):
        self.eps = eps
        self.min_samples = min_samples
        self.labels = None
        self.points = None
        self.colors = None

    def fit_predict(self, points, colors):
        n_points = len(points)
        self.labels = np.full(n_points, -1)  
        self.points = points
        self.colors = colors
        cluster_id = 0
        
        distance_matrix = self._compute_distance_matrix(points)
        
        for i in range(n_points):
            if self.labels[i] != -1: 
                continue  
            
            neighbors = self._get_neighbors(i, distance_matrix)
            
            if len(neighbors) < self.min_samples:
                self.labels[i] = -1  # Mark as noise
            else:
                self._expand_cluster(i, neighbors, cluster_id, distance_matrix)
                cluster_id += 1
        
        return self.get_clusters()

    def _compute_distance_matrix(self, points):
        diff = points[:, np.newaxis] - points  # Compute pairwise differences
        dist_sq = np.sum(diff ** 2, axis=2)  # Compute squared Euclidean distances
        return dist_sq

    def _get_neighbors(self, point_idx, distance_matrix):
        dist_sq = distance_matrix[point_idx]
        neighbors = np.where(dist_sq <= self.eps ** 2)[0]  # Find neighbors within eps distance
        return neighbors

    def _expand_cluster(self, point_idx, neighbors, cluster_id, distance_matrix):
        self.labels[point_idx] = cluster_id
        to_visit = list(neighbors)  # Initialize to_visit with the neighbors
        visited_points = set([point_idx]) 
        
        while to_visit:
            current_point_idx = to_visit.pop()
            if self.labels[current_point_idx] == -1:  # Mark as part of the current cluster
                self.labels[current_point_idx] = cluster_id  
            
            current_neighbors = self._get_neighbors(current_point_idx, distance_matrix)
            
            for neighbor in current_neighbors:
                if self.labels[neighbor] == -1 and neighbor not in visited_points:
                    to_visit.append(neighbor)
                    visited_points.add(neighbor)

    def get_clusters(self):
        clusters = {}
        for idx, label in enumerate(self.labels):
            if label != -1:  
                if label not in clusters:
                    clusters[label] = {'points': [], 'colors': []}
                clusters[label]['points'].append(self.points[idx])  
                clusters[label]['colors'].append(self.colors[idx]) 

        sorted_clusters = {k: v for k, v in sorted(clusters.items(), key=lambda item: len(item[1]['points']), reverse=True)}
        
       
        sorted_cluster_points = {}
        for cluster_id, data in sorted_clusters.items():
            updated_indices = np.arange(len(data['points']))  
            
            sorted_cluster_points[cluster_id] = {
                'points': np.array(data['points'])[updated_indices],  
                'colors': np.array(data['colors'])[updated_indices]  
            }
        
        return sorted_cluster_points


    def get_labels(self):
        return self.labels
