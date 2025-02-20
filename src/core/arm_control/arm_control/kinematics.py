import numpy as np

class Kinematics:
    def __init__(self):
        self.d0 = 0
        self.d1 = 0.001 * 101 
        self.d2 = 0.001 * 95 
        self.d4 = 0.001 * 168

    def get_dh_params(self, joint_angles):
        # theta, d, a, alpha
        return [
            [np.pi, 0,   0,   0],           # Base
            [joint_angles[0], self.d0,   0,        -np.pi/2  ],           # Joint 1
            [-np.pi/ 2 + joint_angles[1], 0,   self.d1,  -np.pi    ],       # Joint 2
            [joint_angles[2], 0,   self.d2,  np.pi     ],       # Joint 3
            [np.pi /2 + joint_angles[3], 0,   0,  np.pi / 2 ],    # Joint 4
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
        return 12000 - (angle / (np.pi) * 18000 )

    def manipulator_encoders_to_angles(self, encoder_values):
        return [self.encoder_to_angle(encoder_value) for encoder_value in encoder_values]

    def manipulator_angles_to_encoders(self, angles):
        return [self.angle_to_encoder(angle) for angle in angles]
