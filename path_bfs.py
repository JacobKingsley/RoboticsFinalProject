#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: Novemeber 15, 2021
# CS 81 Final Project
# path finder BFS function for Robot Movement Controller

""" CONSTANTS """
GRID_SQUARE_OCCUPIED = 100

""" ********** Astar Node class ***************** """
# wrapper node for A* search
class AstarNode:
    def __init__(self, state, heuristic, cost, parent=None):
        self.state = state
        self.heuristic = heuristic
        self.cost = cost
        self.parent = parent


    def priority(self):
        return self.heuristic + self.cost

    def __lt__(self, other):
        return self.priority() < other.priority()

class Grid_Search_Problem:
    # constructor that takes in an occupancy grid and the goal state
    def __init__(self, grid, goal_state):
        self.grid = grid
        self.goal_state = goal_state
    # returns euclidian distance from position in current state and in goal state
    def euclidian_heuristic(self, state):
        return math.sqrt((self.goal_state[0]-state[0])**2 + (self.goal_state[1]-state[1])**2)
    # returns all valid moves from current state
    def get_successors(self, state):
        successors = []

        actions = [(1,0),(1,1),(0,1),(-1, 1),(-1,0),(-1,-1),(0,-1),(1,-1)]

        for action in actions:
            new_pos = (state[0]+action[0], state[1]+action[1])
            if self.grid.is_valid(new_pos[0], new_pos[1]):
                successors.append(new_pos)

        return successors


def astar_search(grid, start, goal):
    # convert start and goal points into cell format for search problem.
    #start_cell = self.grid.get_cell_from_pos(start[0], start[1])
    #goal_cell = self.grid.get_cell_from_pos(goal[0], goal[1])

    start_cell = start
    goal_cell = goal


	if ( (grid.cell_at(goal_grid[0], goal_grid[1]) >= GRID_SQUARE_OCCUPIED) or
			(grid.cell_at(start_grid[0], start_grid[1]) >= GRID_SQUARE_OCCUPIED)):
		# if start or end location on an occupied (wall) space, return empty path
		return []

    # create a search problem using grid and cell index of goal
    search_problem = Grid_Search_Problem(grid, goal_cell)

    start_node = AstarNode(start_cell, search_problem.euclidian_heuristic(start_cell), 0.0)
    pqueue = []
    heappush(pqueue, start_node)

    visited_cost = {}
    visited_cost[start_node.state] = 0

    while pqueue:
        current_node = heappop(pqueue)
        current_state = current_node.state

        if search_problem.goal_test(current_state):
            return backchain(current_node)
        else:
            for child_state in search_problem.get_successors(current_state):
                child_cost = current_node.cost + search_problem.get_cost(current_state, child_state)
                child_node = AstarNode(child_state, search_problem.euclidian_heuristic(child_state), child_cost, current_node)

                if child_state not in visited_cost:
                    heappush(pqueue, child_node)
                    visited_cost[child_state] = child_cost
                elif child_cost < visited_cost[child_state]:
                    heappush(pqueue, child_node)
                    visited_cost[child_state] = child_cost

# back chain through parents of each A* node to get final path
def backchain(node):
    result = []
    current = node
    while current:
        result.append(current.state)
        current = current.parent
    
    result.reverse()
    return result



""" BFS """

def find_path(map, start, goal):
		# takes start and goal coordinates in map frame (units of meters, range is (0, 10))
		# need to convert to occupancy grid resolution (grid squares have length 0.05 m)

		print(start, goal)

		start_grid = map.m_to_grid_coords(start[0], start[1])
		goal_grid = map.m_to_grid_coords(goal[0], goal[1])

		print(start_grid, goal_grid)

		print(map.cell_at(start_grid[0], start_grid[1]))
		print(map.cell_at(goal_grid[0], goal_grid[1]))

		if ( (map.cell_at(goal_grid[0], goal_grid[1]) >= GRID_SQUARE_OCCUPIED) or
				(map.cell_at(start_grid[0], start_grid[1]) >= GRID_SQUARE_OCCUPIED)):
			# if start or end location on an occupied (wall) space, return empty path
			return []


		# run BFS based on grid squares
		x_i = start_grid[0]
		y_i = start_grid[1]

		queue = []
		visited = []
		backtrace_dict = {}

		queue.append((x_i, y_i))
		visited.append((x_i, y_i))
		backtrace_dict[start_grid] = 0

		found = False

		while len(queue) > 0 and not found: # stop the loop if no more unvisited points or the goal is found

			point = queue.pop(0)

			x_i = point[0]
			y_i = point[1]

			for point_new in [(x_i + 1, y_i), (x_i, y_i + 1), (x_i - 1, y_i), (x_i, y_i - 1)]:
				if ((0 <= point_new[0] < map.width) and (0 <= point_new[1] < map.height) and
						(map.cell_at(point_new[0], point_new[1]) < GRID_SQUARE_OCCUPIED) and (point_new not in visited)):
					# if the checked neighbor point is unvisited, unoccupied, and in the grid

					# add point to queue and visted lists
					queue.append(point_new)
					visited.append(point_new)

					# add to dictionary to remember where it came from
					backtrace_dict[point_new] = point

					# check if at goal
					if point_new == goal_grid:
						found = True
						break

		
		# construct a list of the different coordinate points of the path found by the BFS
		# using the backtrace directory to go from the goal back to the start
		# multiply again by the resolution to back to map frame (units of meters)
		path_point = goal_grid
		path = []
		grid_path = []

		while path_point in backtrace_dict:

			m_path_point = map.grid_coords_to_m(path_point[0], path_point[1])
			grid_path.insert(0, path_point)
			path.insert(0, m_path_point)

			path_point = backtrace_dict[path_point]

		print(path)

		return (path, grid_path)