#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: Novemeber 15, 2021
# CS 81 Final Project
# path finder BFS function for Robot Movement Controller



GRID_SQUARE_OCCUPIED = 100

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