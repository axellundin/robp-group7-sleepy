def from_2d_to_3d(x, y, Z, f, c_x=0, c_y=0):
    """
    Convert a 2D point (x, y) to a 3D point (X, Y, Z).
    
    Parameters:
    - x, y: 2D coordinates of the point in the image.
    - f: Focal length of the camera (in pixels).
    - c_x, c_y: The principal point (camera center), usually close to the image center.
    - Z: Depth value (distance from the camera in 3D space).
    
    Returns:
    - (X, Y, Z): 3D point coordinates.
    """
    # Calculate the 3D coordinates using the perspective projection formula
    X = (x - c_x) * Z / f
    Y = (y - c_y) * Z / f
    return X, Y, Z

def from_3d_to_2d(X, Y, Z, f, c_x=0, c_y=0):
    """
    Convert a 3D point (X, Y, Z) to a 2D point (x, y).
    
    Parameters:
    - X, Y, Z: 3D coordinates of the point in space.
    - f: Focal length of the camera (in pixels).
    - c_x, c_y: The principal point (camera center), usually the image center.
    
    Returns:
    - (x, y): 2D coordinates of the point on the image plane.
    """
    # Calculate the 2D coordinates using the perspective projection formula
    x = f * X / Z + c_x
    y = f * Y / Z + c_y
    return x, y
