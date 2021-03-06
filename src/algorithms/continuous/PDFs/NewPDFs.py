import math
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

''' Mathematical Formulas enclosed in these comments '''
def switch_angle_units(value, convert_to='rad'):
	if convert_to == 'rad':
		return math.pi * 2 * (value / 360)
	else:
		return 360 * (value / (2 * math.pi))
def pythagorus(value1, value2):
	return math.sqrt(value1 ** 2 + value2 ** 2)
''' Mathematical formulas above '''

def find_distances(traffic, emergency):
	distances = []
	for pos in traffic:
		x_dist = pos[0] - emergency[0]
		y_dist = pos[1] - emergency[1]
		direct_dist = pythagorus(x_dist, y_dist)
		distances.append(direct_dist)
	return distances
def bearing_from_x_and_y_distance(x_val, y_val):
	if x_val == 0:
		if y_val > 0:
			return 'N'
		else:
			return 'S'
	if y_val == 0:
		if x_val > 0:
			return 'E'
		else:
			return 'W'
	if x_val > 0 and y_val > 0:
		return 'NE'
	if x_val > 0 and y_val < 0:
		return 'SE'
	if x_val < 0 and y_val > 0:
		return 'NW'
	if x_val < 0 and y_val < 0:
		return 'SW'
def bearing_to_angle(bearing, x_val, y_val):
	if bearing == 'N':
		return 0
	if bearing == 'S':
		return 180
	if bearing == 'E':
		return 90
	if bearing == 'W':
		return 270
	temp_angle = abs(math.atan(y_val / x_val))
	temp_angle = switch_angle_units(temp_angle, convert_to='deg')
	if bearing == 'NE':
		angle = 90 - temp_angle
	if bearing == 'SE':
		angle = 90 + temp_angle
	if bearing == 'NW':
		angle = 270 + temp_angle
	if bearing == 'SW':
		angle = (90 - temp_angle) + 180
	return angle
def find_angles(traffic, emergency):
	angles = []
	for pos in traffic:
		x_dist = pos[0] - emergency[0]
		y_dist = pos[1] - emergency[1]
		bearing = bearing_from_x_and_y_distance(x_dist, y_dist)
		angle = bearing_to_angle(bearing, x_dist, y_dist)
		angles.append(angle)
	return angles

''' Within comment are list manipulations '''
def distance_interpolate(distance_list):
	max_val = max(distance_list)
	min_val = min(distance_list)
	new_distance_list = []
	for value in distance_list:
		new_distance_list.append(1 - ((value - min_val) / (max_val - min_val)))
	return new_distance_list
def normalise(list_in):
	total = sum(list_in)
	list_out = []
	for item in list_in:
		list_out.append(item / total)
	return list_out
def invert_probabilities(input_list):
	new_list = []
	for i in input_list:
		new_list.append(1 - i)
	return new_list
def weight_by_max(list_in):
	max_val = max(list_in)
	list_out = []
	for item in list_in:
		list_out.append(item / max_val)
	return list_out
def weight_by_max_and_invert(input_list):
	normalised_list = weight_by_max(input_list)
	inverted_list = invert_probabilities(normalised_list)
	return inverted_list
def split_probabilities(probabilities):
	splitting_factor = int(360 / len(probabilities))
	new_angles = []
	new_probabilities = []
	for i in range(360):
		new_angles.append(i + 1)
	for i in range(len(probabilities)):
		for j in range(splitting_factor):
			new_probabilities.append(probabilities[i])
	return new_angles, new_probabilities
def smear_probabilities(probabilities, degrees_over_to_smear=8):
	new_list = []
	for item in probabilities:
		new_list.append(0)
	for i in range(len(probabilities)):
		value = probabilities[i]
		new_list[i - 4] = new_list[i - 4] + 0.2 * value
		new_list[i - 3] = new_list[i - 3] + 0.4 * value
		new_list[i - 2] = new_list[i - 2] + 0.6 * value
		new_list[i - 1] = new_list[i - 1] + 0.8 * value
		new_list[i] = new_list[i] + value
		if i + 4 >= len(probabilities):
			i = i - len(probabilities)
		new_list[i + 1] = new_list[i + 1] + 0.8 * value
		new_list[i + 2] = new_list[i + 2] + 0.6 * value
		new_list[i + 3] = new_list[i + 3] + 0.4 * value
		new_list[i + 4] = new_list[i + 4] + 0.2 * value
	return new_list
def generate_goal_probabilities(forward, backward):
	new_list = []
	for i in range(360):
		new_list.append(forward)
	for i in range(180):
		new_list[i] = new_list[i] - (i * (forward - backward) / 180)
		new_list[-(i + 1)] = new_list[-(i + 1)] - ((i + 1) * (forward - backward) / 180)
	return new_list
def generate_and_rotate_goal_probabilities(goal_angle, forward, backward):
	general_goal = generate_goal_probabilities(forward, backward)
	rotate_list = math.floor(goal_angle)
	new_list = general_goal[-rotate_list:] + general_goal[:-rotate_list]
	return new_list
def raise_to_the_power_probabilities(list_in, index):
	list_out = []
	for item in list_in:
		list_out.append(item**index)
	return list_out
''' List manipulation above '''

def sweep_angles(precision, distances, angles):
	current_angle = 0
	sweeping_angles_list = []
	tot_distances_list = []
	while current_angle < 360:
		tot_distances = 0
		i = 0
		for item in angles:
			if current_angle <= item < current_angle + precision:
				tot_distances = tot_distances + distances[i]
			i = i + 1
		tot_distances_list.append(tot_distances)
		sweeping_angles_list.append(current_angle + precision)
		current_angle = current_angle + precision
	return tot_distances_list, sweeping_angles_list

def shaded_polygons(precision, emergency, world_width, world_length):
	current_angle = 0
	triangle_coords = []
	R = 5 * math.sqrt(world_width ** 2 + world_length ** 2)
	temp_precision = switch_angle_units(precision, convert_to='rad')
	while current_angle < 360:
		temp_cur_angle = switch_angle_units(current_angle, convert_to='rad')
		triangle_coords.append([
			emergency,
			[emergency[0] + R * math.sin(temp_cur_angle), emergency[1] + R * math.cos(temp_cur_angle)],
			[emergency[0] + R * math.sin(temp_cur_angle + temp_precision),
			 emergency[1] + R * math.cos(temp_cur_angle + temp_precision)]
		])
		current_angle = current_angle + precision
	return triangle_coords
def get_colours(input_list, num_colour):
	presets = []
	for i in range(num_colour):
		presets.append(((1-(i/(num_colour-1))),i/(num_colour-1),0))

	max_val = max(input_list)
	min_val = min(input_list)
	input_list_2 = []
	for item in input_list:
		input_list_2.append((item-min_val)/(max_val-min_val))
	input_list = input_list_2

	colours = []
	for item in input_list:
		index = math.floor(num_colour * item)
		if index == num_colour:
			index = num_colour - 1
		colours.append(presets[index])

	return colours

def get_infographic_cars(traffic_pose_list, emergency_pose, precision_in, a_g, p_f, p_b, no_of_colours, brightness, e_colour, e_marker, t_colour, t_marker, power):

	y_coords = []
	x_coords = []
	for item in traffic_pose_list:
		y_coords.append(item[1])
		x_coords.append(item[0])
	max_y = max(y_coords)
	min_y = min(y_coords)
	max_x = max(x_coords)
	min_x = min(x_coords)

	w_width = max_x - min_x
	w_length = max_y - min_y

	temp_angle_list, probability_list = raw_positions_to_probability_sweep(traffic_pose_list, emergency_pose, precision_in, a_g, p_f, p_b, power)

	fig, ax = plt.subplots()

	colours = get_colours(probability_list, no_of_colours)
	# polygons = shaded_polygons(sweep_precision, emergency)
	polygons = shaded_polygons(1, emergency_pose, w_width, w_length)
	for i in range(len(polygons)):
		poly = mpatches.Polygon(polygons[i], closed=True, alpha=brightness, color=colours[i], ec=None)
		ax.add_patch(poly)

	x_to_plot = []
	y_to_plot = []
	for item in traffic_pose_list:
		x_to_plot.append(item[0])
		y_to_plot.append(item[1])

	ax.scatter(x_to_plot, y_to_plot, color=t_colour, marker=t_marker)
	ax.scatter(emergency_pose[0], emergency_pose[1], color=e_colour, marker=e_marker)
	ax.set_xlim(min_x-5, max_x+5)
	ax.set_ylim(min_y-5, max_y+5)
	plt.show()
def get_graph_of_pdf(traffic_pose_list, emergency_pose, precision_in, a_g, p_f, p_b, power):

	angle_list, probability_list = raw_positions_to_probability_sweep(traffic_pose_list, emergency_pose, precision_in, a_g, p_f, p_b, power)

	plt.style.use('ggplot')

	sweeped_angles_enumerated = [i for i, _ in enumerate(angle_list)]

	plt.bar(sweeped_angles_enumerated, probability_list, color='green')
	plt.xlabel("Angle from North, Clockwise")
	plt.ylabel("Probability to move")
	plt.title("Probability to move at a given angle")

	plt.xticks(sweeped_angles_enumerated, angle_list)

	plt.show()



def raw_positions_to_probability_sweep(car_positions, emergency, sweep_precision, goal_angle, forward_probability, backward_probability, power=1):
	distances = find_distances(car_positions, emergency)
	weighted_distances = distance_interpolate(distances)
	angles = find_angles(car_positions, emergency)

	summed_distances, sweeped_angles = sweep_angles(sweep_precision, weighted_distances, angles)
	summed_distances = normalise(summed_distances)
	probability_values = weight_by_max_and_invert(summed_distances)
	sweeped_angles, probability_values = split_probabilities(probability_values)
	probability_values = smear_probabilities(probability_values)
	probability_values = weight_by_max(probability_values)

	goal_list = generate_and_rotate_goal_probabilities(goal_angle, forward_probability, backward_probability)
	combined_probabilities = []
	for i in range(360):
		combined_probabilities.append(probability_values[i] * goal_list[i])
	combined_probabilities = weight_by_max(combined_probabilities)
	probability_values = combined_probabilities
	probability_values = raise_to_the_power_probabilities(probability_values, power)
	probability_values = normalise(probability_values)
	probability_values = weight_by_max(probability_values)

	return sweeped_angles, probability_values

def get_prob_from_angle(traffic_pose_list, emergency_pose, test_angle, p_f, p_b, a_g, precision_in, power):
	temp_angle_list, temp_probability_list = raw_positions_to_probability_sweep(traffic_pose_list, emergency_pose, precision_in, a_g, p_f, p_b, power)
	test_angle_index = math.floor(test_angle)
	return temp_probability_list[test_angle_index]




def angle_probability_from_pdf(traffic_pose_list, emergency_pose, heading, angle_to_check):
	goal_angle = -heading
	precision = 10
	f_prob = 1
	b_prob = 0
	colours_in_infographic = 20
	opacity = 0.2
	emergency_colour = 'red'
	emergency_marker = 'x'
	traffic_colour = 'black'
	traffic_marker = 'x'
	exp = 5

	# get_infographic_cars(traffic_pose_list, emergency_pose, precision, goal_angle, f_prob, b_prob, colours_in_infographic, opacity, emergency_colour, emergency_marker, traffic_colour, traffic_marker, exp)

	prob = get_prob_from_angle(traffic_pose_list, emergency_pose, angle_to_check, f_prob, b_prob, goal_angle, precision, exp)
	return prob

