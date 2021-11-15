#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: November 15, 2021
# CS 81 Final Project
# Function library to help interpret paths for Robot Movement Controller

# takes a litst of points 
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


	return movements_list

		

		


def find_direction(start_x, start_y, end_x, end_y):
	if abs(end_x - start_x) >= 1:
		return 'x'
	elif abs(end_y - start_y) >= 1:
		return 'y'
	
