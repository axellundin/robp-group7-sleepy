import numpy as np

class RRT:
    def __init__(self, joint_limits, start_joint_angle, goal_pose, collision_free_fn, kinematics_fn, pose_goal_error_fn, goal_threshold=0.01, step_size=0.04, max_iterations=1000):
        self.start_joint_angle = start_joint_angle
        self.goal_pose = goal_pose
        self.collision_free_fn = collision_free_fn
        self.joint_limits = joint_limits
        self.kinematics_fn = kinematics_fn
        self.pose_goal_error_fn = pose_goal_error_fn
        self.nodes = [self.start_joint_angle]  # List of node configurations
        self.parents = [None]  # List of parent indices (None for root)
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_threshold = goal_threshold
        self.closest_distance = np.inf
        self.closest_node = None
    
    def sample_random_joint_angle(self):
        """
        Sample a random joint angle from the joint limits
        """
        return np.random.uniform(self.joint_limits[:, 0], self.joint_limits[:, 1])
    
    def sample_around_closest_node(self):
        """
        Sample a random joint angle around the closest node
        """
        # Construct a joint limit space that is a certain radius around the closest node 
        joint_limit_space = np.zeros(self.joint_limits.shape)
        radius = 0.01 * self.step_size  # radius around each joint angle
        joint_limit_space[:, 0] = np.maximum(self.joint_limits[:, 0], self.closest_node - radius)
        joint_limit_space[:, 1] = np.minimum(self.joint_limits[:, 1], self.closest_node + radius)
        return np.random.uniform(joint_limit_space[:, 0], joint_limit_space[:, 1])

    def compute_closest_node(self, random_joint_angle):
        """
        Compute the closest node in the tree to the random joint angle
        """
        distances = np.linalg.norm(np.array(self.nodes) - random_joint_angle, axis=1)
        return np.argmin(distances)
    
    def steer(self, closest_node, random_joint_angle):
        """
        Steer from the closest node to the random joint angle
        """
        step_len = 0.05 * self.step_size + np.random.uniform(0, 1) * 0.95 * self.step_size
        return closest_node + (random_joint_angle - closest_node) * step_len

    def _iterate(self):
        """
        Iterate the RRT algorithm
        """
        random_joint_angle =  self.sample_around_closest_node() if self.closest_distance < 0.05 else self.sample_random_joint_angle()
        closest_idx = self.compute_closest_node(random_joint_angle)
        closest_node = self.nodes[closest_idx]
        new_joint_angle = self.steer(closest_node, random_joint_angle)
        T = self.kinematics_fn(new_joint_angle)

        pose = np.array([T[0, 3], T[1, 3], T[2, 3], 
                         np.arctan2(T[2, 1], np.sqrt(T[0, 1]**2 + T[1, 1]**2)), 
                         np.arctan2(-T[0, 2], np.sqrt(T[1, 2]**2 + T[2, 2]**2)), 
                         np.arctan2(T[1, 2], T[2, 2])])
        
        if self.collision_free_fn(pose[0], pose[1], pose[2]):
            self.nodes.append(new_joint_angle)
            self.parents.append(closest_idx)

        if self.pose_goal_error_fn(pose, self.goal_pose) < self.closest_distance:
            print(f"New closest distance: {self.pose_goal_error_fn(pose, self.goal_pose)}")
            self.closest_distance = self.pose_goal_error_fn(pose, self.goal_pose)
            self.closest_node = new_joint_angle

        if self.closest_distance < self.goal_threshold:
            goal_angles = new_joint_angle
            return True, goal_angles
        return False, None

    def run(self):
        for _ in range(self.max_iterations):
            success, goal_angles = self._iterate()
            if success:
                return True, goal_angles
        return False, None
    