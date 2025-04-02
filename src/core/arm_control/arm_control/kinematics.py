import numpy as np

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Kinematics:
    def __init__(self, joint_limits):
        self.d0 = 0
        self.d1 = 0.001 * 101 
        self.d2 = 0.001 * 95 
        self.d4 = 0.001 * 168
        self.joint_limits = joint_limits

    def get_dh_params(self, joint_angles):
        # theta, d, a, alpha
        return [
            [np.pi, 0, 0, 0],           # Base
            [joint_angles[0], self.d0,   0,        -np.pi/2  ],           # Joint 1
            [-np.pi/ 2 + joint_angles[1], 0,   self.d1,  -np.pi    ],       # Joint 2
            [joint_angles[2], 0,   self.d2,  np.pi     ],       # Joint 3
            [np.pi /2 + joint_angles[3], 0,   0,  np.pi/2 ],    # Joint 4
            [0,  self.d4, 0,        0        ]                  # End Effector
        ]
    
    def dh_matrix(self, theta, d, a, alpha):
        """Calculate transformation matrix from DH parameters"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def analytical_solution(self, rho, z, theta, phi, alpha):
        """ 
        Analytical solution for the inverse kinematics of the manipulator
        Args:
            rho: radial distance from the origin to the point in the x-y plane
            z: vertical distance from the origin to the point
            theta0: x-y plane angle of the base of the manipulator
            phi: angle of the end-effector in the x-y plane
        Returns:
            joint_angles: list of joint angles [theta0, theta1, theta2, theta3, theta4]
        """
        print("rho: ", rho, "z: ", z, "theta0: ", theta, "phi: ", phi, "alpha: ", alpha)
        zp = z + self.d4 * np.cos(alpha)
        print("zp: ", zp)
        r = rho - self.d4 * np.sin(alpha)
        L = np.sqrt(r**2 + zp**2)
        beta = -np.arccos( (self.d1**2 + L**2 - self.d2**2) / (2 * L * self.d1) )
        # print("beta: ", beta)
        flag = np.sign(1- L / self.d2 * np.cos(beta))
        theta1 = -np.pi/2  + np.arctan2(zp, r) - beta
        theta2 = np.pi * (flag > 0) + flag * np.arcsin( L / self.d2 * np.sin(beta) )
        theta3 = np.pi - theta1 + theta2 + alpha
        theta4 = phi + theta 

        angles = [theta, theta1, theta2, theta3, theta4]
        clipped_angles = []

        for i, a in enumerate(angles): 
            angle = wrap_to_pi(a)
            if angle > self.joint_limits[i][1]: 
                print("joint ", i, "angle ", angle, "clipped to ", self.joint_limits[i][1])
                angle = self.joint_limits[i][1] 
            elif angle < self.joint_limits[i][0]: 
                print("joint ", i, "angle ", angle, "clipped to ", self.joint_limits[i][0])
                angle = self.joint_limits[i][0] 
            clipped_angles.append(angle)
        
        return clipped_angles
    
    def cartesian_to_cylindrical(self, x, y, z):
        rho = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        return rho, z, theta
        

    def compute_jacobian(self, joint_angles):
        """Compute the Jacobian matrix for the current robot configuration"""
        # Initialize 6x5 Jacobian matrix (6 DOF end-effector, 5 joints)
        jacobian = np.zeros((6, 5))
        current_transform = np.eye(4)
        end_effector = self.get_joint_position(joint_angles, 6)
        dh_params = self.get_dh_params(joint_angles)
        # Only compute for the 5 joints (skip base and end-effector frames)
        z_axis = np.array([0, 0, 1])
        joint_pos = np.array([0, 0, 0])
        for i in range(5):
            # Get the current joint's z-axis
            transform = self.dh_matrix(dh_params[i][0], 
                                        dh_params[i][1],
                                        dh_params[i][2], 
                                        dh_params[i][3])
            current_transform = current_transform @ transform
            z_axis = current_transform[0:3, 2]
            joint_pos = current_transform[0:3, 3]
            
            # Position of end-effector
            end_pos = end_effector[0:3, 3]
            
            # Compute linear velocity component (cross product)
            pos_diff = end_pos - joint_pos
            linear_vel = np.cross(z_axis, pos_diff)
            
            
            # Angular velocity component is just the z-axis for revolute joints
            angular_vel = z_axis
            
            # Fill in the Jacobian column
            jacobian[0:3, i] = linear_vel
            jacobian[3:6, i] = angular_vel
        
        return jacobian

    def inverse_kinematics(self, joint_angles, target_pose, max_iterations=10000, tolerance=1e-3):
        """
        Solve inverse kinematics using the Jacobian pseudo-inverse method
        Args:
            target_pose: 4x4 homogeneous transformation matrix of desired end-effector pose
            max_iterations: maximum number of iterations for the algorithm
            tolerance: convergence tolerance
        Returns:
            (success, joint_angles): tuple containing success flag and final joint angles
        """
        current_joints = np.array(joint_angles, dtype=np.float64)
        alpha = 0.5 # Step size
        
        for iteration in range(max_iterations):
            current_pose = self.get_joint_position(current_joints, 6)
            print("current_pose: ", current_pose)
            print("target_pose: ", target_pose)
            # Position error
            pos_error = target_pose[0:3, 3] - current_pose[0:3, 3]
            
            # Orientation error
            R_current = current_pose[0:3, 0:3]
            R_target = target_pose[0:3, 0:3]
            R_error = R_target @ R_current.T
            
            # Convert to axis-angle representation
            rot_error = np.zeros(3)
            rot_error[0] = R_error[2, 1] - R_error[1, 2]
            rot_error[1] = R_error[0, 2] - R_error[2, 0]
            rot_error[2] = R_error[1, 0] - R_error[0, 1]
            rot_error *= 0.5 
            
            # Combined error vector with different weights
            pos_gain = 1.0
            rot_gain = 0.3
            error = np.concatenate([pos_gain * pos_error, rot_gain * rot_error])
            
            # Check both position and orientation convergence
            test1 = np.linalg.norm(pos_error) < tolerance
            test2 = np.linalg.norm(rot_error) < tolerance * 10
            print("pos_error: ", pos_error, "norm: ", np.linalg.norm(pos_error), "passes test? ", test1)
            print("rot_error: ", rot_error, "norm: ", np.linalg.norm(rot_error), "passes test? ", test2)
            if test1 and test2:
                return True, current_joints
            
            # Calculate Jacobian
            J = self.compute_jacobian(current_joints)
            
            # Add damping for numerical stability
            cond_number = np.linalg.cond(J)
            lambda_ = 0.001 if cond_number < 50 else 0.01  # Increase λ for ill-conditioned Jacobians
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_ * np.eye(6))
            
            # Calculate joint adjustments
            delta_theta = alpha * J_pinv @ error
            
            # Update joint angles with limits
            new_joints = current_joints + delta_theta
            for i in range(5):
                new_joints[i] = np.clip(new_joints[i], self.joint_limits[i][0], self.joint_limits[i][1])
            
            # Check if we're making progress
            if np.linalg.norm(new_joints - current_joints) < 1e-6:
                print("Reason: No progress, iteration: ", iteration)
                print("Current joint angles: ", current_joints)
                print("Error: ", error, "norm: ", np.linalg.norm(error))
                print("Pos error: ", pos_error, "norm: ", np.linalg.norm(pos_error))
                print("Rot error: ", rot_error, "norm: ", np.linalg.norm(rot_error))
                print("Target pose: ", target_pose)
                print("Current pose: ", current_pose)
                print("J: ", J)
                print("conditioning number: ", np.linalg.cond(J))
                return False, current_joints
            
            current_joints = new_joints
            joint_angles = current_joints.tolist()
        
        # If we reach here, we didn't converge
        print("Reason: Max iterations reached, iteration: ", max_iterations)
        print("Current joint angles: ", current_joints)
        print("Error: ", error, "norm: ", np.linalg.norm(error))
        print("Pos error: ", pos_error, "norm: ", np.linalg.norm(pos_error))
        print("Rot error: ", rot_error, "norm: ", np.linalg.norm(rot_error))
        print("Target pose: ", target_pose)
        print("Current pose: ", current_pose)
        print("J: ", J)
        print("conditioning number: ", np.linalg.cond(J))
        return False, current_joints

    def get_joint_position(self, joint_angles, joint_idx):
        dh_params = self.get_dh_params(joint_angles)
        transform = np.eye(4)
        for i in range(joint_idx):
            transform = transform @ self.dh_matrix(dh_params[i][0], dh_params[i][1], dh_params[i][2], dh_params[i][3])
        return transform

    def fw_kinematics(self, joint_angles):
        """Get the current end-effector pose"""
        transform = np.eye(4)
        dh_params = self.get_dh_params(joint_angles)
        for param in dh_params:
            transform = transform @ self.dh_matrix(param[0], param[1], param[2], param[3])
        return transform
    
    def encoder_to_angle(self, encoder_value): 
        return (encoder_value - 12000) / 18000 * np.pi

    def angle_to_encoder(self, angle):
        return 12000 + (angle / (np.pi) * 18000 )

    def manipulator_encoders_to_angles(self, encoder_values):
        return [self.encoder_to_angle(encoder_value) for encoder_value in encoder_values]

    def manipulator_angles_to_encoders(self, angles):
        return [self.angle_to_encoder(angle) for angle in angles]
