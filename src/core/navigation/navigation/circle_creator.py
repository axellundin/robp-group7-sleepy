import numpy as np

class circle_creator:
    def __init__(self, resolution):
        self.resolution = resolution

    # Creates a circle around a point, either the whole area or just the edge, and either fills in or returns all grids,
    # either only within the area or across the entire grid map
    def circle_filler_angle_dependent(self, x,y,area,grid, radius,fill_in,edge_only, area_only, from_middle, v1, v2):

        x_start = 0
        y_start = 0
        x_end = grid.shape[1]-1
        y_end = grid.shape[0]-1

        if area_only:
            x_start = area.x_start
            y_start = area.y_start
            x_end = area.x_end
            y_end = area.y_end
        
        obs_expand_percentage = 0.1
        start_angles_obs1 = []
        end_angles_obs1 = []
        start_angles_obs2 = []
        end_angles_obs2 = []
        start_angles_obs3 = []
        end_angles_obs3 = []
        start_angles_obs4 = []
        end_angles_obs4 = []
        if edge_only == False and  grid[y,x] < 0.1 and grid[y,x] > - 0.1:
            point_on_circle = [[x,y]]
            if fill_in == True:
                grid[y,x] = 1
        else:
            point_on_circle = []
  
        outer_detection_cell_distance = int(np.ceil(radius/self.resolution))
        inner_detection_cell_distance = outer_detection_cell_distance
        if from_middle == True:
            inner_detection_cell_distance = 2
        for detection_cell_distance in range(inner_detection_cell_distance,outer_detection_cell_distance+1):
            if edge_only == True:
                point_on_circle = []

            dist_to_midpoint_x = detection_cell_distance-1
            dist_to_midpoint_y = 0
            if fill_in == True:
                fill_in_nr = np.float64(1.0)
                pre_fill_in_nr = np.float64(1.0)
            else:
                fill_in_nr =  np.float64(0.000001*x + 0.0000000001*y + 0.000000000001*detection_cell_distance)
                pre_fill_in_nr =  np.float64(0.000001*x + 0.0000000001*y + 0.000000000001*(detection_cell_distance-1))


            obs1 = False
            obs2 = False
            obs3 = False
            obs4 = False

            while True:
                obs_expand_y = (obs_expand_percentage+ 0.02*np.abs((dist_to_midpoint_x-dist_to_midpoint_y)/(detection_cell_distance-1)))*self.resolution*(dist_to_midpoint_x)/(dist_to_midpoint_y+dist_to_midpoint_x)
                obs_expand_x = (obs_expand_percentage+ 0.02*np.abs((dist_to_midpoint_x-dist_to_midpoint_y)/(detection_cell_distance-1)))*self.resolution*(dist_to_midpoint_y)/(dist_to_midpoint_y+dist_to_midpoint_x)
                stop1 = False
                stop2 = False
                stop3 = False
                stop4 = False
                for i in range(len(end_angles_obs1)):
                    if np.arctan2(dist_to_midpoint_y, -dist_to_midpoint_x) >= end_angles_obs1[i] and np.arctan2(dist_to_midpoint_y, -dist_to_midpoint_x) <= start_angles_obs1[i]:
                        stop1 = True
                for i in range(len(end_angles_obs2)):
                    if np.arctan2(-dist_to_midpoint_y, -dist_to_midpoint_x) <= end_angles_obs2[i] and np.arctan2(-dist_to_midpoint_y, -dist_to_midpoint_x) >= start_angles_obs2[i]:
                        stop2 = True
                for i in range(len(end_angles_obs3)):
                    if np.arctan2(dist_to_midpoint_y, dist_to_midpoint_x) <= end_angles_obs3[i] and np.arctan2(dist_to_midpoint_y, dist_to_midpoint_x) >= start_angles_obs3[i]:
                        stop3 = True
                for i in range(len(end_angles_obs4)):
                    if np.arctan2(-dist_to_midpoint_y, dist_to_midpoint_x) >= end_angles_obs4[i] and np.arctan2(-dist_to_midpoint_y, dist_to_midpoint_x) <= start_angles_obs4[i]:
                        stop4 = True

                # Nagative x
                if x - dist_to_midpoint_x >= x_start:
                    # Positive y
                    if y+dist_to_midpoint_y <= y_end and grid[y+dist_to_midpoint_y, x - dist_to_midpoint_x] == -1:
                        if obs1 == False:
                            if dist_to_midpoint_y == 0:
                                start_angles_obs1.append(np.pi)
                            else:
                                start_angles_obs1.append(np.arctan2(dist_to_midpoint_y-obs_expand_y, -dist_to_midpoint_x-obs_expand_x))
                            obs1 = True
                    elif y+dist_to_midpoint_y <= y_end and grid[y+dist_to_midpoint_y, x - dist_to_midpoint_x] == 1:
                        if edge_only == True:
                            point_on_circle.append([x - dist_to_midpoint_x, y+dist_to_midpoint_y])
                        if obs1 == True:
                            end_angles_obs1.append(np.arctan2(prev_dist_to_midpoint_y+obs_expand_y, -prev_dist_to_midpoint_x+obs_expand_x))
                            obs1 = False
                    elif y+dist_to_midpoint_y <= y_end and y+dist_to_midpoint_y >= y_start and grid[y+dist_to_midpoint_y, x - dist_to_midpoint_x] != pre_fill_in_nr and stop1 == False:
                        grid[y+dist_to_midpoint_y, x - dist_to_midpoint_x] = fill_in_nr
                        point_on_circle.append([x - dist_to_midpoint_x, y+dist_to_midpoint_y])
                        if obs1 == True:
                            end_angles_obs1.append(np.arctan2(prev_dist_to_midpoint_y+obs_expand_y, -prev_dist_to_midpoint_x+obs_expand_x))
                            obs1 = False

                    # Negative y
                    if y-dist_to_midpoint_y >= y_start and grid[y-dist_to_midpoint_y, x - dist_to_midpoint_x] == -1:
                        if obs2 == False:
                            if dist_to_midpoint_y == 0:
                                start_angles_obs2.append(-np.pi)
                            else:
                                start_angles_obs2.append(np.arctan2(-dist_to_midpoint_y+obs_expand_y, -dist_to_midpoint_x-obs_expand_x))
                            obs2 = True
                    elif y-dist_to_midpoint_y >= y_start and grid[y-dist_to_midpoint_y, x - dist_to_midpoint_x] == 1:
                        if edge_only == True:
                            point_on_circle.append([x - dist_to_midpoint_x, y-dist_to_midpoint_y])
                        if obs2 == True:
                            end_angles_obs2.append(np.arctan2(-prev_dist_to_midpoint_y-obs_expand_y, -prev_dist_to_midpoint_x+obs_expand_x))
                            obs2 = False
                    elif y-dist_to_midpoint_y <= y_end and y-dist_to_midpoint_y >= y_start and dist_to_midpoint_y != 0 and grid[y-dist_to_midpoint_y, x - dist_to_midpoint_x] != pre_fill_in_nr and stop2 == False:
                        grid[y-dist_to_midpoint_y, x - dist_to_midpoint_x] = fill_in_nr
                        point_on_circle.append([x - dist_to_midpoint_x, y-dist_to_midpoint_y])
                        if obs2 == True:
                            end_angles_obs2.append(np.arctan2(-prev_dist_to_midpoint_y-obs_expand_y, -prev_dist_to_midpoint_x+obs_expand_x))
                            obs2 = False

                # Positive x
                if x + dist_to_midpoint_x <= x_end and dist_to_midpoint_x != 0:
                    # Positive y
                    if y+dist_to_midpoint_y <= y_end and grid[y+dist_to_midpoint_y, x + dist_to_midpoint_x] == -1:
                        if obs3 == False:
                            if dist_to_midpoint_y == 0:
                                start_angles_obs3.append(0)
                            else:
                                start_angles_obs3.append(np.arctan2(dist_to_midpoint_y-obs_expand_y, dist_to_midpoint_x+obs_expand_x))
                            obs3 = True
                    elif y+dist_to_midpoint_y <= y_end and grid[y+dist_to_midpoint_y, x + dist_to_midpoint_x] == 1:
                        if edge_only == True:
                            point_on_circle.append([x + dist_to_midpoint_x, y+dist_to_midpoint_y])
                        if obs3 == True:
                            end_angles_obs3.append(np.arctan2(prev_dist_to_midpoint_y+obs_expand_y,prev_dist_to_midpoint_x-obs_expand_x))
                            obs3 = False
                    elif y+dist_to_midpoint_y <= y_end and y+dist_to_midpoint_y >= y_start and grid[y+dist_to_midpoint_y, x + dist_to_midpoint_x] != pre_fill_in_nr and stop3 == False:
                        grid[y+dist_to_midpoint_y, x + dist_to_midpoint_x] = fill_in_nr
                        point_on_circle.append([x + dist_to_midpoint_x, y+dist_to_midpoint_y])
                        if obs3 == True:
                            end_angles_obs3.append(np.arctan2(prev_dist_to_midpoint_y+obs_expand_y,prev_dist_to_midpoint_x-obs_expand_x))
                            obs3 = False

                    # Negative y
                    if y-dist_to_midpoint_y >= y_start and grid[y-dist_to_midpoint_y, x + dist_to_midpoint_x] == -1:
                        if obs4 == False:
                            if dist_to_midpoint_y == 0:
                                 start_angles_obs4.append(0)
                            else:
                                start_angles_obs4.append(np.arctan2(-dist_to_midpoint_y+obs_expand_y,dist_to_midpoint_x+obs_expand_x))
                            obs4 = True
                    elif y-dist_to_midpoint_y >= y_start and grid[y-dist_to_midpoint_y, x + dist_to_midpoint_x] == 1:
                        if edge_only == True:
                            point_on_circle.append([x + dist_to_midpoint_x, y-dist_to_midpoint_y])
                        if obs4 == True:
                            end_angles_obs4.append(np.arctan2(-prev_dist_to_midpoint_y-obs_expand_y,prev_dist_to_midpoint_x-obs_expand_x))
                            obs4 = False
                    elif y-dist_to_midpoint_y <= y_end and y-dist_to_midpoint_y >= y_start and dist_to_midpoint_y != 0 and grid[y-dist_to_midpoint_y, x + dist_to_midpoint_x] != pre_fill_in_nr and stop4 == False:
                        grid[y-dist_to_midpoint_y, x + dist_to_midpoint_x] = fill_in_nr
                        point_on_circle.append([x + dist_to_midpoint_x, y-dist_to_midpoint_y])
                        if obs4 == True:
                            end_angles_obs4.append(np.arctan2(-prev_dist_to_midpoint_y-obs_expand_y,prev_dist_to_midpoint_x-obs_expand_x))
                            obs4 = False

                if dist_to_midpoint_x == 0:
                    break
                prev_dist_to_midpoint_x = dist_to_midpoint_x
                prev_dist_to_midpoint_y = dist_to_midpoint_y
                if np.sqrt((dist_to_midpoint_x)**2 + (dist_to_midpoint_y+1)**2) > detection_cell_distance:
                    dist_to_midpoint_x -= 1
                    
                else:
                    dist_to_midpoint_y += 1
            if obs1 == True:
                end_angles_obs1.append(np.pi/2)
                start_angles_obs3.append(np.arctan2(dist_to_midpoint_y, obs_expand_x))
                end_angles_obs3.append(np.pi/2)
                obs1 = False
            if obs2 == True:
                end_angles_obs2.append(-np.pi/2)
                start_angles_obs4.append(np.arctan2(-dist_to_midpoint_y, obs_expand_x))
                end_angles_obs4.append(-np.pi/2)
                obs2 = False
            if obs3 == True:
                end_angles_obs3.append(np.pi/2)
                obs3 = False
            if obs4 == True:
                end_angles_obs4.append(-np.pi/2)
                obs4 = False

        return point_on_circle