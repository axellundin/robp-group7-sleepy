import numpy as np
from math import cos, sin, atan2, fabs

# One grid cell
class Node:
    def __init__(self, tot_cost, parent_cost, x, y, parent):
        self.parent = parent
        self.tot_cost = tot_cost
        self.parent_cost = parent_cost
        self.x = x
        self.y = y
        
# cost_weighting: 
# determines how much priority should be given to driving towards the goal 
# versus driving from the start. A value of 1.0 means equal weighting, while 1.1 means a
# higher weighting towards driving towards the goal
# direct_distance_start:
# determines whether the cost to the start should be calculated as the direct (Euclidean)
# distance or as a discrete step-based distance
# diagonal_cost:
# specifies the cost for diagonal movement in discrete movement
class path_planner:
    def __init__(self):
        self.grid = None
        self.x_values = []
        self.y_values = []
        
    def A_star(self, grid, start, goal, cost_weighting, direct_distance_start, diagonal_cost, direct_distance_goal):
        succeed = False
        self.x_values = []
        self.y_values = []
        self.grid = grid
        [start_x, start_y] = start
        [goal_x,goal_y] = goal

        start_tot_cost = cost_weighting*np.sqrt((start_x-goal_x)**2+(start_y-goal_y)**2)
        start_node = Node(start_tot_cost, 0, start_x, start_y, None)
        queue = [start_node]

        grid_hight = self.grid.shape[0]
        grid_width = self.grid.shape[1]

        #Loop until queue is empty
        while queue:
            #print(f"Inne i A star while loop: len queue: {len(queue)}")

            #Retrieve the node at the front of the queue
            current_node = queue.pop(0)
            current_x = current_node.x
            current_y = current_node.y

            if current_x == goal_x and current_y == goal_y:
                succeed = True
                break

            current_parent_cost = current_node.parent_cost
            tmp_node_list = []
            obs_in_x = True
            obs_in_neg_x = True

            # Check for obstacles and creates new nodes if there is no obstacle in that direction and if it has not already been added

            # Adds a node in positive x direction
            if current_x+1 < grid_width and (self.grid[current_y,current_x+1] == 0 or self.grid[current_y,current_x+1] == 0.25):
                self.grid[current_y,current_x+1] = 0.5
                parent_cost = current_parent_cost + 1
                if direct_distance_start == True:
                    if direct_distance_goal == True:
                        tot_cost = cost_weighting*np.sqrt(((current_x+1)-goal_x)**2+(current_y-goal_y)**2) + np.sqrt(((current_x+1)-start_x)**2+(current_y-start_y)**2)
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x+1)-goal_x)+np.abs(current_y-goal_y)) + np.sqrt(((current_x+1)-start_x)**2+(current_y-start_y)**2)
                else:
                    if direct_distance_goal == True:
                        tot_cost = cost_weighting*np.sqrt(((current_x+1)-goal_x)**2+(current_y-goal_y)**2) + parent_cost
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x+1)-goal_x)+np.abs(current_y-goal_y)) + parent_cost
                tmp_node_list.append(Node(tot_cost, parent_cost, current_x+1, current_y, current_node))
                obs_in_x = False

            # Adds a node in negative x direction
            if current_x-1 >= 0 and (self.grid[current_y,current_x-1] == 0 or self.grid[current_y,current_x-1] == 0.25):
                self.grid[current_y,current_x-1] = 0.5
                parent_cost = current_parent_cost + 1
                if direct_distance_start == True:
                    if direct_distance_goal == True:
                        tot_cost = cost_weighting*np.sqrt(((current_x-1)-goal_x)**2+(current_y-goal_y)**2) + np.sqrt(((current_x-1)-start_x)**2+(current_y-start_y)**2)
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x-1)-goal_x)+np.abs(current_y-goal_y))  + np.sqrt(((current_x-1)-start_x)**2+(current_y-start_y)**2)
                else:
                    if direct_distance_goal == True:            
                        tot_cost = cost_weighting*np.sqrt(((current_x-1)-goal_x)**2+(current_y-goal_y)**2) + parent_cost
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x-1)-goal_x)+np.abs(current_y-goal_y)) + parent_cost
                tmp_node_list.append(Node(tot_cost, parent_cost, current_x-1, current_y, current_node))
                obs_in_neg_x = False
            
            # Adds a node in positive y direction
            if current_y+1 < grid_hight and (self.grid[current_y+1,current_x] == 0 or self.grid[current_y+1,current_x] == 0.25):
                self.grid[current_y+1,current_x] = 0.5
                parent_cost = current_parent_cost + 1
                if direct_distance_start == True:
                    if direct_distance_goal == True:  
                        tot_cost = cost_weighting*np.sqrt(((current_x)-goal_x)**2+((current_y+1)-goal_y)**2) + np.sqrt(((current_x)-start_x)**2+((current_y+1)-start_y)**2)
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x)-goal_x)+np.abs((current_y+1)-goal_y)) + np.sqrt(((current_x)-start_x)**2+((current_y+1)-start_y)**2)
                else:
                    if direct_distance_goal == True:            
                        tot_cost = cost_weighting*np.sqrt(((current_x)-goal_x)**2+((current_y+1)-goal_y)**2) + parent_cost
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x)-goal_x)+np.abs((current_y+1)-goal_y)) + parent_cost
                tmp_node_list.append(Node(tot_cost, parent_cost, current_x, current_y+1, current_node))

                # Adds a node in diagonal, positive x and y direction
                if obs_in_x == False and (self.grid[current_y+1,current_x+1] == 0 or self.grid[current_y+1,current_x+1] == 0.25):
                    self.grid[current_y+1,current_x+1] = 0.5
                    parent_cost = current_parent_cost + diagonal_cost
                    if direct_distance_start == True:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x+1)-goal_x)**2+((current_y+1)-goal_y)**2) + np.sqrt(((current_x+1)-start_x)**2+((current_y+1)-start_y)**2)
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x+1)-goal_x)+np.abs((current_y+1)-goal_y)) + np.sqrt(((current_x+1)-start_x)**2+((current_y+1)-start_y)**2)
                    else:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x+1)-goal_x)**2+((current_y+1)-goal_y)**2) + parent_cost
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x+1)-goal_x)+np.abs((current_y+1)-goal_y)) + parent_cost
                    tmp_node_list.append(Node(tot_cost, parent_cost, current_x+1, current_y+1, current_node))
                
                # Adds a node in diagonal, negative x positive y direction
                if obs_in_neg_x == False and (self.grid[current_y+1,current_x-1] == 0 or self.grid[current_y+1,current_x-1] == 0.25):
                    self.grid[current_y+1,current_x-1] = 0.5
                    parent_cost = current_parent_cost + diagonal_cost
                    if direct_distance_start == True:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x-1)-goal_x)**2+((current_y+1)-goal_y)**2) + np.sqrt(((current_x-1)-start_x)**2+((current_y+1)-start_y)**2)
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x-1)-goal_x)+np.abs((current_y+1)-goal_y)) + np.sqrt(((current_x-1)-start_x)**2+((current_y+1)-start_y)**2)
                    else:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x-1)-goal_x)**2+((current_y+1)-goal_y)**2) + parent_cost
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x-1)-goal_x)+np.abs((current_y+1)-goal_y)) + parent_cost
                    tmp_node_list.append(Node(tot_cost, parent_cost, current_x-1, current_y+1, current_node))
            
            # Adds a node in negative y direction
            if current_y-1 >= 0 and (self.grid[current_y-1,current_x] == 0 or self.grid[current_y-1,current_x] == 0.25):
                self.grid[current_y-1,current_x] = 0.5
                parent_cost = current_parent_cost + 1
                if direct_distance_start == True:
                    if direct_distance_goal == True:
                        tot_cost = cost_weighting*np.sqrt(((current_x)-goal_x)**2+((current_y-1)-goal_y)**2) + np.sqrt(((current_x)-start_x)**2+((current_y-1)-start_y)**2)
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x)-goal_x)+np.abs((current_y-1)-goal_y)) + np.sqrt(((current_x)-start_x)**2+((current_y-1)-start_y)**2)
                else:
                    if direct_distance_goal == True:            
                        tot_cost = cost_weighting*np.sqrt(((current_x)-goal_x)**2+((current_y-1)-goal_y)**2) + parent_cost
                    else:
                        tot_cost = cost_weighting*(np.abs((current_x)-goal_x)+np.abs((current_y-1)-goal_y)) + parent_cost
                tmp_node_list.append(Node(tot_cost, parent_cost, current_x, current_y-1, current_node))

                # Adds a node in diagonal, positive x negative y direction
                if obs_in_x == False and (self.grid[current_y-1,current_x+1] == 0 or self.grid[current_y-1,current_x+1] == 0.25):
                    #print("sjunde")
                    self.grid[current_y-1,current_x+1] = 0.5
                    parent_cost = current_parent_cost + diagonal_cost
                    if direct_distance_start == True:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x+1)-goal_x)**2+((current_y-1)-goal_y)**2) + np.sqrt(((current_x+1)-start_x)**2+((current_y-1)-start_y)**2)
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x+1)-goal_x)+np.abs((current_y-1)-goal_y)) + np.sqrt(((current_x+1)-start_x)**2+((current_y-1)-start_y)**2)
                    else:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x+1)-goal_x)**2+((current_y-1)-goal_y)**2) + parent_cost
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x+1)-goal_x)+np.abs((current_y-1)-goal_y)) + parent_cost
                    tmp_node_list.append(Node(tot_cost, parent_cost, current_x+1, current_y-1, current_node))

                # Adds a node in diagonal, negative x negative y direction
                if obs_in_neg_x == False and (self.grid[current_y-1,current_x-1] == 0 or self.grid[current_y-1,current_x-1] == 0.25):
                    #print("åttonde")
                    self.grid[current_y-1,current_x-1] = 0.5
                    parent_cost = current_parent_cost + diagonal_cost
                    if direct_distance_start == True:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x-1)-goal_x)**2+((current_y-1)-goal_y)**2) + np.sqrt(((current_x-1)-start_x)**2+((current_y-1)-start_y)**2)
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x-1)-goal_x)+np.abs((current_y-1)-goal_y)) + np.sqrt(((current_x-1)-start_x)**2+((current_y-1)-start_y)**2)
                    else:
                        if direct_distance_goal == True:
                            tot_cost = cost_weighting*np.sqrt(((current_x-1)-goal_x)**2+((current_y-1)-goal_y)**2) + parent_cost
                        else:
                            tot_cost = cost_weighting*(np.abs((current_x-1)-goal_x)+np.abs((current_y-1)-goal_y)) + parent_cost
                    tmp_node_list.append(Node(tot_cost, parent_cost, current_x-1, current_y-1, current_node))

            # inserts the new nodes in the queue
            for node in tmp_node_list:
                i = 0
                while i < len(queue) and queue[i].tot_cost < node.tot_cost:
                    i +=1
                queue.insert(i, node)
        if succeed == False:
            return None, False
        while current_node != None:
            self.grid[current_node.y, current_node.x] = 70
            self.x_values.append(current_node.x)
            self.y_values.append(current_node.y)
            current_node = current_node.parent
        
        return self.grid, True
    
    def waypoint_creator(self, rgbd, grid):
        #print('Creating waypoints to next exploration position')
        x_values = self.x_values.copy()
        y_values = self.y_values.copy()
        waypoints = []
        if len(x_values) < 3:
            waypoints.append((x_values[-1], y_values[-1])) 
        else:
            next_waypoint = [None,None]
            last_point_x = x_values[0]
            last_point_y = y_values[0]
            x_values.pop(-1)
            y_values.pop(-1)
            while len(x_values) != 0 and not(next_waypoint[0] == last_point_x and next_waypoint[1] == last_point_y):
                valid_waypoint = "succes"
                start_point = (x_values.pop(-1), y_values.pop(-1))
                next_waypoint = start_point
                if len(x_values) != 0:
                    next_waypoint = (x_values.pop(-1), y_values.pop(-1))
                while len(x_values) != 0 and valid_waypoint == "succes":
                    test_waypoint = (x_values[-1], y_values[-1])
                    valid_waypoint, gbd_fail = self.check_waypoint(start_point, test_waypoint, rgbd, grid)
                    if gbd_fail == True:
                        waypoints.append(next_waypoint)
                        made_it_all_the_way = False
                        return waypoints, made_it_all_the_way
                    if valid_waypoint == "succes":
                        next_waypoint = (x_values.pop(-1), y_values.pop(-1))
                waypoints.append(next_waypoint)
        #print(f'Number of waypoints: {len(waypoints)}')
        made_it_all_the_way = True
        return waypoints, made_it_all_the_way

    def check_waypoint(self, start, end, rgbd, grid):

        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)+1):
            if grid[int(y),int(x)] == -1 :
                rgbd_fail = False
                return "fail", rgbd_fail
            if (rgbd == True and grid[int(y),int(x)] == 0):
                rgbd_fail = True
                return "fail", rgbd_fail

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                    if grid[int(y),int(x)] == -1:
                        rgbd_fail = False
                        return "fail", rgbd_fail
                    if (rgbd == True and grid[int(y),int(x)] == 0):
                        rgbd_fail = True
                        return "fail", rgbd_fail
                y += y_inc
                error += dx
        gbd_fail = False
        return "succes", gbd_fail