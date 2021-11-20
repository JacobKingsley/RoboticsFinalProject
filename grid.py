#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: November 15, 2021
# CS 81 Final Project
# Grid class for Robot Movement Controller

# true division
from __future__ import division

# Import of python modules.
import numpy as np
import copy


GRID_SQUARE_OCCUPIED = 100

# manually set the scale of the inflation depending on the particular map
# The higher the scale, the further from the wall the robot stays
INFLATE_SCALE = 7


class Grid:
	def __init__(self, occupancy_grid_data, width, height, resolution, origin):
		
		self.grid = np.reshape(occupancy_grid_data, (height, width))
		self.resolution = resolution
		self.width = width
		self.height = height
		self.origin = origin

	def set_point(self, x, y, occupancy=100):
		# method used to set a point in the grid. Adds the inputted point parameters
		# to the origin to center the gird around a given point
		if 0 <= x < self.width and 0 <= y < self.height:
			self.grid[y, x] = occupancy

	def is_valid(self, x, y):
		# print("r")
		# print(not (0 <= x < self.width and 0 <= y < self.height))
		if not (0 <= x < self.width and 0 <= y < self.height):
			return False

		# print("occu")
		# print(self.cell_at(x, y) >= GRID_SQUARE_OCCUPIED)
		if self.cell_at(x, y) >= GRID_SQUARE_OCCUPIED or self.cell_at(x, y) == -1:
			return False

		# print("true")
		return True

	def cell_at(self, x, y):
		return self.grid[y, x]

	def m_to_grid_coords(self, x, y):
		x_coord = int(round( (x - self.origin.position.x) / self.resolution))
		y_coord = int(round( (y - self.origin.position.y) / self.resolution))
		return (x_coord, y_coord)

	def grid_coords_to_m(self, x, y):
		x_m = x * self.resolution + self.origin.position.x
		y_m = y * self.resolution + self.origin.position.y
		return (x_m, y_m)

	def inflate_copy(self):

		new_map = copy.deepcopy(self)
		for x in range(self.width):
			for y in range(self.height):

				if self.cell_at(x, y) >= GRID_SQUARE_OCCUPIED:
					# set points around occupied points as occupied
					for i in range(-1 * INFLATE_SCALE//2, INFLATE_SCALE//2 + 1):
						for j in range(-1 * INFLATE_SCALE, INFLATE_SCALE + 1):
							
							if new_map.cell_at(i, j) < GRID_SQUARE_OCCUPIED:
								new_map.set_point(x + i, y + j, 100)

				elif self.cell_at(x, y) < GRID_SQUARE_OCCUPIED and new_map.cell_at(x, y) == -1:
					new_map.set_point(x, y, self.cell_at(x, y))

		return new_map

	# returns the grid as a 1D list like is needed in the occupancy grid publisher message
	def grid_as_list(self):
		return list(self.grid.flatten())