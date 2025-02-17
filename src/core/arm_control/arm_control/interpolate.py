import numpy as np
from scipy.spatial.transform import Rotation, Slerp

def sample_bezier_curve(p0, p1, p2, p3, num_points):
    """
    Sample points along a cubic Bezier curve.
    
    Args:
        p0: Start point
        p1, p2: Control points
        p3: End point
        num_points: Number of points to sample (including start and end points)
    
    Returns:
        Array of shape (num_points, 3) containing sampled points
    """
    # Generate parameter values from 0 to 1 (inclusive)
    t = np.linspace(0, 1, num_points)
    
    # Cubic Bezier formula
    points = (
        (1 - t)[:, np.newaxis] ** 3 * p0 +
        3 * (1 - t)[:, np.newaxis] ** 2 * t[:, np.newaxis] * p1 +
        3 * (1 - t)[:, np.newaxis] * t[:, np.newaxis] ** 2 * p2 +
        t[:, np.newaxis] ** 3 * p3
    )
    
    return points

def interpolate_positions(start_pos, end_pos, start_quat=None, end_quat=None, num_points=50):
    """
    Create a smooth path between two positions and orientations using cubic Bezier 
    interpolation for positions and SLERP for orientations. The path's tangent
    direction will match the orientation at start and end points.
    
    Args:
        start_pos: Starting position as (x, y, z) tuple or array
        end_pos: Ending position as (x, y, z) tuple or array
        start_quat: Starting orientation as (x, y, z, w) quaternion (optional)
        end_quat: Ending orientation as (x, y, z, w) quaternion (optional)
        num_points: Number of points to generate along the path (default: 50)
    
    Returns:
        If orientations provided:
            Tuple (positions, orientations) where:
            - positions: Array of shape (num_points, 3) containing path points
            - orientations: Array of shape (num_points, 4) containing quaternions
        If no orientations:
            Array of shape (num_points, 3) containing path points
    """
    # Convert position inputs to numpy arrays
    start_pos = np.array(start_pos)
    end_pos = np.array(end_pos)
    
    if start_quat is not None and end_quat is not None:
        # Get direction vectors from quaternions
        start_rot = Rotation.from_quat(start_quat)
        end_rot = Rotation.from_quat(end_quat)
        
        # Use the x-axis direction as the tangent
        start_dir = start_rot.apply([1, 0, 0])
        end_dir = end_rot.apply([1, 0, 0])
        
        # Scale the direction vectors based on the path length
        path_length = np.linalg.norm(end_pos - start_pos)
        control_dist = path_length * 0.3
        
        # Create control points that enforce the orientation
        control_1 = start_pos + start_dir * control_dist
        control_2 = end_pos - end_dir * control_dist
    else:
        # Fallback to previous behavior if no orientations provided
        direction = end_pos - start_pos
        control_1 = start_pos + direction * 0.3
        control_2 = start_pos + direction * 0.7
    
    # Sample points along the Bezier curve
    positions = sample_bezier_curve(start_pos, control_1, control_2, end_pos, num_points)
    
    # If no orientations provided, return only positions
    if start_quat is None or end_quat is None:
        return positions
    
    # Generate parameter values from 0 to 1 for SLERP
    t = np.linspace(0, 1, num_points)
    
    # Handle orientations using SLERP
    key_rots = Rotation.from_quat([start_quat, end_quat])
    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)
    
    # Interpolate orientations
    orientations = slerp(t).as_quat()
    
    return positions, orientations



