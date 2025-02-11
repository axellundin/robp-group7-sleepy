from polygon import Polygon
import numpy as np
import matplotlib.pyplot as plt

def discretize(polygon: Polygon, resolution: float, angular_resolution: float,base_padding: float,robot_convex_hull: list[np.ndarray]) -> list[np.ndarray]:
    # Get the bounding box of the polygon
    min_x = min(vertex[0] for vertex in polygon.verticies)
    max_x = max(vertex[0] for vertex in polygon.verticies)
    min_y = min(vertex[1] for vertex in polygon.verticies)
    max_y = max(vertex[1] for vertex in polygon.verticies) 

    # Create a grid of the given resolution
    grid_x = np.arange(min_x, max_x, resolution)
    grid_y = np.arange(min_y, max_y, resolution)
    grid_th = np.arange(-np.pi, np.pi, angular_resolution)

    # Compute a version of the robot convex hull rotated by each angle in grid_th
    # Extend the robot convex hull by the base padding the direction corresponding to each vertex
    robot_convex_hull = [vertex + base_padding * np.array([np.cos(np.arctan2(vertex[1], vertex[0])), np.sin(np.arctan2(vertex[1], vertex[0]))]) for vertex in robot_convex_hull]

    # Rescale by spatial resolution
    robot_convex_hull_rotated = []
    verticies = np.array(robot_convex_hull)
    for th in grid_th:
        rotation_matrix = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        rotated_verticies = np.dot(verticies, rotation_matrix)
        robot_convex_hull_rotated.append(rotated_verticies)
    # Create a grid of the given resolution
    grid = np.meshgrid(grid_x, grid_y, grid_th)
    occupancy = np.zeros(grid[0].shape)

    for i in range(grid[0].shape[0]):
        for j in range(grid[0].shape[1]):
            for k in range(grid[0].shape[2]):
                position = np.array([grid[0][i, j, k], grid[1][i, j, k]])
                no_intersections = True
                if not polygon.is_internal(position):
                    occupancy[i, j, k] = 0
                    continue
                for l in range(len(robot_convex_hull_rotated[k])):
                    start = position + robot_convex_hull_rotated[k][l] 
                    end = position + robot_convex_hull_rotated[k][(l + 1) % len(robot_convex_hull_rotated[k])]
                    result = not polygon.segment_intersects_polygon(start,end)
                    # print(f"start: {start}, end: {end}, result: {result}")
                    no_intersections = no_intersections and result
                    if not no_intersections:
                        break
                occupancy[i, j, k] = no_intersections

    return occupancy

if __name__ == "__main__":
    verticies = [
        np.array([0.0, 0.0]),     # Start bottom left
        np.array([5.0, 0.0]),     # Bottom wall
        np.array([5.0, 5.0]),     # Right wall
        np.array([0.0, 5.0]),     # Top wall
        np.array([0.0, 4.0]),     # First corridor
        np.array([4.0, 4.0]),
        np.array([4.0, 3.0]),
        np.array([1.0, 3.0]),
        np.array([1.0, 2.0]),     # Middle section
        np.array([4.0, 2.0]),
        np.array([4.0, 1.0]),
        np.array([0.5, 1.0]),
        np.array([0.5, 0.5]),     # Entry corridor
        np.array([0.0, 0.5]),
    ]
    
    robot_convex_hull = [
        np.array([0.05, 0]),
        np.array([0.05, 0.13]),
        np.array([-0.12, 0.13]),
        np.array([-0.25, 0.13]),
        np.array([-0.25, 0.0]),
        np.array([-0.25, -0.13]),
        np.array([-0.12, -0.13]),
        np.array([0.05, -0.13]),
    ]
    robot_convex_hull = [vertex * 0.2 for vertex in robot_convex_hull]  # Made robot slightly larger
    
    # Create a polygon from the vertices
    polygon = Polygon(verticies)
    res = 0.05  # Increased resolution for larger map
    discretized = discretize(polygon, res, np.pi / 16, 0.02, robot_convex_hull)
    # Display the discretized grid
    # show a slice of the discretized grid
    plt.imshow(discretized[:,:,16], cmap='gray')

    # show the robot convex hull
    for i in range(len(robot_convex_hull)):
        plt.plot(robot_convex_hull[i][0] / res, robot_convex_hull[i][1] / res, 'r.-') 
 
    # show the polygon
    vertices_array = np.array(verticies + [verticies[0]])  # Add first vertex at end to close polygon
    plt.plot(vertices_array[:, 0] / res, vertices_array[:, 1] / res, 'b-', label='Polygon')
    plt.show()
    np.save("occupancy_grid.npy", discretized)
