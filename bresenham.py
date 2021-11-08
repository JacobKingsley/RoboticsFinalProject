class Test:
	def __init__(self):
		pass

	def bresenham(self, grid, x1, y1, x2, y2, wall=True):
		# output: (list empty, tuple occupied)
		# 
		# 	empty:
		# 		A list of tuples containing (int x, int y) representing
		# 		coordinate points in thr grid frame of the points along
		# 		the ray that are unoccupied
		# 
		# 	occupied:
		# 		A tuple formatted (int x, int y) holding the coordinate
		# 		at the end of the ray in grid coordinates that is either
		# 		the point on the object detected or is located a distance
		# 		away equal to the max range of the sensor. In the former
		# 		case the point is occupied, in the latter case the
		# 		point is unoccupied
		#
		# takes the grid to be edited, the start and end points of a line
		# segment in grid square coordinates, and whether the point (x2, y2)
		# is a wall (occupied point) or empty
		dx = x2 - x1
		dy = y2 - y1

		# if dx = 0, its a vertical line and the points need to be incremented along the y coordinates
		# if |m| > 1 (or |m| is infinity), swap coordinates, compute, and then swap back
		# so that algorithm can increment along the y-coordinates instead of the x
		if dx == 0 or abs(dy / dx) > 1:
			# run algorithm with swapped x and y for |m| > 1
			(empty, occupied) = self.bresenham_helper(y1, x1, y2, x2)
			# swap coordinates back
			for i in range(len(empty)):
				empty[i] = (empty[i][1], empty[i][0])
			occupied = (occupied[1], occupied[0])
			
		else:	# if |m| <= 1, run normally
			(empty, occupied) = self.bresenham_helper(x1, y1, x2, y2)

		# now that we have the points along the line segment, set their occupancy value as appropriate
		# 0 for empty spaces, 100 for occupied spaces
		for pt in empty:
			# TODO: ADD POINT TO GRID
			pass
			# grid.set_point(pt[0], pt[1], 0)
		
		# if no_wall flag is on, then the laser detected max distance
		# so point at the end of the ray should be unoccupied
		# TODO: ADD END OF RAY POINT TO GRID AS EITHER OCCUPIED OR EMPTY
		if wall:
			pass
			# grid.set_point(occupied[0], occupied[1], 100)
		else:
			pass
			# grid.set_point(occupied[0], occupied[1], 0)

		return empty, occupied


	def bresenham_helper(self, x1, y1, x2, y2):
		# helper function for bresenham function that takes in start point (x1, y1) and end point (x2, y2), and the slope m
		# 
		# output: (list empty, tuple occupied)
		# 
		# 	empty:
		# 		A list of tuples containing (int x, int y) representing
		# 		coordinate points in thr grid frame of the points along
		# 		the ray that are unoccupied
		# 
		# 	occupied:
		# 		A tuple formatted (int x, int y) holding the coordinate
		# 		at the end of the ray in grid coordinates that is either
		# 		the point on the object detected or is located a distance
		# 		away equal to the max range of the sensor. In the former
		# 		case the point is occupied, in the latter case the
		# 		point is unoccupied
		#

		m = (y2 - y1) / (x2 - x1)

		# list holding output points along line segment that are not the endpoint
		empty = []

		# holds cumulative error as we increment
		e = 0

		# start point for the y value
		y = y1

		# find directions that x and y increment in
		if x2 > x1:
			ddx = 1
		else:
			ddx = -1

		if y2 > y1:
			ddy = 1
		else:
			ddy = -1

		for x in range(x1, x2+ddx, ddx):
			# add to the list of empty points
			empty.append((x, y))
			if (e + (m*ddx)) * ddy < 0.5:
				e += (m*ddx)
			else:
				e += (m*ddx) - ddy
				y += ddy

		# last point is an occupied point
		occupied = (x2, y2)

		return empty, occupied





t = Test()
e = t.bresenham(None, 10, 2, 0, 5)[0]
print(e)