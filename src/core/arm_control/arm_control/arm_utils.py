import numpy as np

def get_dh_params(joint_angles, d0, d1, d2, d4):
    # theta, d, a, alpha
    return [
        [0, 0,   0,       0 ],           # Base
        [joint_angles[0], d0,   0,        -np.pi/2  ],           # Joint 1
        [-np.pi/ 2 + joint_angles[1], 0,   d1,  -np.pi    ],       # Joint 2
        [joint_angles[2], 0,   d2,  np.pi     ],       # Joint 3
        [np.pi /2 + joint_angles[3], 0,   0,  np.pi / 2 ],    # Joint 4
        [0,  d4, 0,        0        ]                  # End Effector
    ]
    
def dh_matrix(theta, d, a, alpha):
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

def get_end_effector_pose(joint_angles, d0, d1, d2, d4):
    """Get the current end-effector pose"""
    transform = np.eye(4)
    dh_params = get_dh_params(joint_angles, d0, d1, d2, d4)
    for param in dh_params:
        transform = transform @ dh_matrix(param[0], param[1], param[2], param[3])
    return transform

def compute_jacobian(joint_angles, d0, d1, d2, d4):
    """Compute the Jacobian matrix for the current robot configuration"""
    # Initialize 6x5 Jacobian matrix (6 DOF end-effector, 5 joints)
    jacobian = np.zeros((6, 5))
    current_transform = np.eye(4)
    end_effector = get_end_effector_pose(joint_angles, d0, d1, d2, d4)
    dh_params = get_dh_params(joint_angles, d0, d1, d2, d4)
    # Only compute for the 5 joints (skip base and end-effector frames)
    for i in range(5):
        # Get the current joint's z-axis
        if i == 0:
            z_axis = np.array([0, 0, 1])
            joint_pos = np.array([0, 0, 0])
        else:
            transform = dh_matrix(dh_params[i][0], 
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

def inverse_kinematics(joint_angles, target_pose, d0, d1, d2, d4, max_iterations=10000, tolerance=1e-3):
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
    

    joint_limits = [
        (-np.pi * 120 / 180 , np.pi * 240 / 120),      # Joint 1
        (-np.pi / 2, np.pi/4),        # Joint 2 (adjusted for -np.pi/2 offset)
        (-np.pi/2, np.pi/2),      # Joint 3
        (-np.pi/2, np.pi/2),          # Joint 4 (wider range for wrist)
        (-np.pi * 120 / 180 , np.pi * 240 / 120),      # Joint 5
    ]

    # # Original joint limits
    # joint_limits = [
    #     (-np.pi/2, np.pi/2),      # Joint 1
    #     (-np.pi/2, np.pi/2),      # Joint 2
    #     (-np.pi/2, np.pi/2),      # Joint 3
    #     (-np.pi/2, np.pi/2),      # Joint 4
    #     (-np.pi/2, np.pi/2),      # Joint 5
    # ]
    
    for iteration in range(max_iterations):
        current_pose = get_end_effector_pose(current_joints, d0, d1, d2, d4)
        
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
        rot_error *= 0.5 * 0
        
        # Combined error vector with different weights
        pos_gain = 1.0
        rot_gain = 0.3
        error = np.concatenate([pos_gain * pos_error, rot_gain * 0 * rot_error])
        
        # Check both position and orientation convergence
        if np.linalg.norm(pos_error) < tolerance and np.linalg.norm(rot_error) < tolerance * 10:
            return True, current_joints
        
        # Calculate Jacobian
        J = compute_jacobian(current_joints, d0, d1, d2, d4)
        
        # Add damping for numerical stability
        cond_number = np.linalg.cond(J)
        lambda_ = 0.001 if cond_number < 50 else 0.01  # Increase λ for ill-conditioned Jacobians
        J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_ * np.eye(6))
        
        # Calculate joint adjustments
        delta_theta = alpha * J_pinv @ error
        
        # Update joint angles with limits
        new_joints = current_joints + delta_theta
        for i in range(5):
            new_joints[i] = np.clip(new_joints[i], joint_limits[i][0], joint_limits[i][1])
        
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

def get_joint_position(joint_angles, d0, d1, d2, d4, joint_idx):
    """Get the position of a specific joint"""
    dh_params = get_dh_params(joint_angles, d0, d1, d2, d4)
    transform = np.eye(4)
    for i in range(joint_idx):
        transform = transform @ dh_matrix(dh_params[i][0], 
                                                dh_params[i][1],
                                                dh_params[i][2], 
                                                dh_params[i][3])
    return transform

def encoder_to_angle(encoder_value): 
    return (encoder_value - 12000) / 18000 * np.pi

def angle_to_encoder(angle):
    return 12000 + angle / (np.pi) * 18000 

def manipulator_encoders_to_angles(encoder_values, joint_cfgs):
    angles = []
    for encoder_value, joint_cfg in zip(encoder_values, joint_cfgs): 
        joint_limits = joint_cfg['limits'] 
        joint_offset = joint_cfg['offset'] 

        if encoder_value < joint_limits[0] or encoder_value > joint_limits[1]: 
            encoder_val = joint_limits[0] if encoder_value < joint_limits[0] else joint_limits[1]
            angle = encoder_to_angle(encoder_val)
            angles.append(angle)
            continue
        
        angle = encoder_to_angle(encoder_value + joint_offset)
        angles.append(angle)

    return [encoder_to_angle(encoder_value) for encoder_value in encoder_values]

def manipulator_angles_to_encoders(angles, joint_cfgs):
    encoders = []
    for angle, joint_cfg in zip(angles, joint_cfgs): 
        joint_limits = joint_cfg['limits'] 
        joint_offset = joint_cfg['offset'] 

        encoder_val = angle_to_encoder(angle)

        if encoder_val < joint_limits[0] or encoder_val > joint_limits[1]: 
            encoder_val = joint_limits[0] if encoder_val < joint_limits[0] else joint_limits[1]
            encoders.append(encoder_val)
            continue
        
        encoder_val = angle_to_encoder(angle) + joint_offset
        encoders.append(encoder_val)

    return encoders