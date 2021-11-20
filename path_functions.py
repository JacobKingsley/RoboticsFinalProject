#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: November 15, 2021
# CS 81 Final Project
# Function library to help interpret paths for Robot Movement Controller

import random

GRID_SQUARE_OCCUPIED = 100

# generates random points on a map until the point is on an unoccupied grid square,
# in which case it is a valid end point
def generate_waypoint(inflated_map):
	width = inflated_map.width
	height = inflated_map.height

	rand_x = int(round(width * random.random()))
	rand_y = int(round(height * random.random()))

	while not inflated_map.is_valid(rand_x, rand_y):
		rand_x = int(round(width * random.random()))
		rand_y = int(round(height * random.random()))

	return (rand_x, rand_y)


# takes a litst of points and returns the list of movements
# that is required for the robot to travel along the path
# returns a tuple (char direction, int distance) where direction is either 'x' or 'y'
# and distance is the length of the motion of that line segment in grid square units
def get_grid_movements(grid_path):

	curr_d = find_direction(grid_path[0][0], grid_path[0][1], grid_path[1][0], grid_path[1][1])

	curr_x = grid_path[1][0]
	curr_y = grid_path[1][1]


	movements_list = []

	curr_movement_l = 1


	# if abs((path[0] - x)) < path
	for point in grid_path[2:]:

		new_d = find_direction(curr_x, curr_y, point[0], point[1])

		if new_d == curr_d:
			if new_d == 'x':
				curr_movement_l += (point[0] - curr_x)
			elif new_d == 'y':
				curr_movement_l += (point[1] - curr_y)

		else:
			movements_list.append((curr_d, curr_movement_l))
			curr_d = new_d
			curr_movement_l = 1


		curr_x = point[0]
		curr_y = point[1]

	# add last movement to list
	movements_list.append((curr_d, curr_movement_l))

	return movements_list

		

		
# helper function for get_grid_movements() that checks the
# direction traveled from one point to the next in the path list
def find_direction(start_x, start_y, end_x, end_y):
	if abs(end_x - start_x) >= 1:
		return 'x'
	elif abs(end_y - start_y) >= 1:
		return 'y'
	

### BFS algorithm to find the path between two points on the grid
def find_path(map, start_grid, goal_grid):
	# takes start and goal coordinates in grid frame

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

	return grid_path
