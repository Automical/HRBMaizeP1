# Config
self.num_points = 50 # Number of points to generate as initial particles
self.pf_gen_radius = 10 # Radius in custom coordinates around current location to gen points
self.point_weight_cutoff = 0.01 # Throw away particles with weights below this cutoff
self.loc_assign_threshold = 0.8 # Only assign current location highest weighted particle if 
				# the weight is >= this threshold 

self.current_location = [-1, -1]
self.points = []
self.point_weights = []

"""
DESCRIPTION: Check if input coordinates are within radius around current_location
"""
def within_circle(x, y)
	diff_x = self.current_location[0] - x
	diff_y = self.current_location[1] - y
	return (np.sqrt(np.power(diff_x, 2) + np.power(diff_y, 2)) <= self.pf_gen_radius)

"""
DESCRIPTION: Regenerate a random field of points in a circle around current_location (with 			equal weights) based on "self.num_point"
INPUTS: None
REQUIRES: self.current_location to be set by waypoint 0 by the time this func is called
MODIFIES: self.points, self.point_weights
OUTPUTS: None
"""
def reset_pf()
	# Set current location to points with highest weight
	if len(self.points_weights) > 0: # Else keep current loc unchanged
		highest_weight = 0
		corresponding_index = 0
		for i in range(len(self.point_weights))
			if self.points_weights[i] > highest_weight:
				highest_weight = self.points_weights[i]
				corresponding_index = i
		if highest_weight >= loc_assign_threshold: # Else keep current loc unchanged
			self.current_loc = self.points[corresponding_index]

	# Clear current points and their weights
	self.points = []
	self.point_weights = []
	
	# Generate a field of possible points around current location
	for pt in range(self.num_points):
		x_noise = np.random.randint(-self.pf_gen_radius, self.pf_gen_radius)
		gen_x = self.current_location[0] + x_noise
		y_noise = np.random.randint(-self.pf_gen_radius, self.pf_gen_radius)
		gen_y = self.current_location[1] + y_noise
		while (not self.within_circle(gen_x, gen_y)):
			x_noise = np.random.randint(-self.pf_gen_radius, self.pf_gen_radius)
			gen_x = self.current_location[1] + x_noise
			y_noise = np.random.randint(-self.pf_gen_radius, self.pf_gen_radius)
			gen_y = self.current_location[1] + y_noise
		# Found points in the circle
		self.points.append([gen_x, gen_y])
		self.point_weights.append([1.0 / self.num_points])
	
"""
DESCRIPTION:
INPUTS: front_sensor_dist - Orthogonal distance from front_sensor to line to follow to the 			next waypoint in integers 0 to 255, larger values means closer to the line 
	back_sensor_dist - same description as "front_sensor_dist" but with the back sensor
MODIFIES: None
OUTPUTS: None
"""
def estimated_location(way_1, way_2, front_sensor_dist, back_sensor_dist)
	# Find equation of line between way_1 and way_2
	A = way_2[1] - way_1[1] / way_2[0] - way_1[0] # Slope
	B = 1
	C = way_1[1] - A * way_1[0]

	#Update point weights TODO: Improve update method to used past weights
	for i in range(len(self.points)):
		dist = np.abs(A * points[i][0] + B * points[i][1] + C)
		dist /= np.sqrt(np.power(A, 2) + np.power(B, 2))
		
		sensor_dist = 0
		if (front_sensor_dist > 0 and back_sensor_dist > 0)
			sensor_dist = (front_sensor_dist + back_sensor_dist) / 2.0
		elif (front_sensor_dist > 0)
			sensor_dist = front_sensor_dist
		elif (back_sensor_dist > 0)
			sensor_dist = back_sensor_dist
		else
			sensor_dist = dist

		self.point_weights[i] = np.abs(dist - sensor_dist) / 255.0 # max diff is 

	#Eliminate low weight points
	kept_indicies = self.point_weights >= self.point_weight_cutoff

	self.points = self.points[kept_indicies]
	self.point_weights = self.points_weights[kepts_indicies]







		