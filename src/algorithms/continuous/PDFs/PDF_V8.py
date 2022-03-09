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

def find_angles(traffic, emergency):
	angles = []
	for pos in traffic:
		x_dist = pos[0] - emergency[0]
		y_dist = pos[1] - emergency[1]
		bearing = bearing_from_x_and_y_distance(x_dist, y_dist)
		angle = bearing_to_angle(bearing, x_dist, y_dist)
		angles.append(angle)
	return angles
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
def probabilites_reorganise(input_list):
	max_val = max(input_list)
	normalised_list = []
	for item in input_list:
		normalised_list.append(item/max_val)
	inverted_list = []
	for item in normalised_list:
		inverted_list.append(1-item)
	return inverted_list
def split_probabilities(probabilities):
	splitting_factor = int(360/len(probabilities))
	new_angles = []
	new_probabilities = []
	for i in range(360):
		new_angles.append(i+1)
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
def final_interpolate(input_list):
	max_val = max(input_list)
	new_list = []
	for item in input_list:
		new_list.append(item/max_val)
	return new_list
def generate_goal_probabilities(forward, backward):
	new_list = []
	for i in range(360):
		new_list.append(1)
	for i in range(180):
		new_list[i] = new_list[i] - (i*(forward - backward)/180)
		new_list[-(i+1)] = new_list[-(i+1)] - ((i+1)*(forward - backward)/180)
	return new_list
def rotate_goal_probabilites(goal_angle, forward, backward):
	general_goal = generate_goal_probabilities(forward, backward)
	rotate_list = math.floor(goal_angle)
	new_list = general_goal[-rotate_list:]+general_goal[:-rotate_list]
	return new_list
''' List manipulation above '''

def sweep_angles(precision, distances, angles):
	current_angle = 0
	sweeping_angles_list = []
	tot_distances_list = []
	while current_angle < 360:
		tot_distances = 0
		for item in angles:
			i = 0
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
			[emergency[0] + R * math.sin(temp_cur_angle + temp_precision), emergency[1] + R * math.cos(temp_cur_angle + temp_precision)]
		])
		current_angle = current_angle + precision
	return triangle_coords

def get_colours(input_list):
	presets = [
		'red',
		'orangered',
		'darkorange',
		'gold',
		'yellow',
		'palegoldenrod',
		'yellowgreen',
		'limegreen',
		'mediumspringgreen',
		'lime'
	]
	colours = []
	for item in input_list:
		index = math.floor(10*item)
		if index == 10:
			index = 9
		colours.append(presets[index])
	return colours

def main_function_to_run(car_positions, emergency, sweep_precision):
	distances = find_distances(car_positions, emergency)
	weighted_distances = distance_interpolate(distances)
	angles = find_angles(car_positions, emergency)

	summed_distances, sweeped_angles = sweep_angles(sweep_precision, weighted_distances, angles)
	probability_values = normalise(summed_distances)
	probability_values = probabilites_reorganise(probability_values)
	sweeped_angles, probability_values = split_probabilities(probability_values)
	probability_values = smear_probabilities(probability_values)
	probability_values = final_interpolate(probability_values)
	return sweeped_angles, probability_values

def get_prob_from_angle(traffic_pose_list, emergency_pose, angle):
	temp_precision = 10
	temp_angle_list, temp_probability_list = main_function_to_run(traffic_pose_list, emergency_pose, temp_precision)
	test_angle = math.floor(angle)
	return temp_probability_list[test_angle]

traffic = [
	[2, 25],
	[6, 28],
	[10, 27],
	[10, 21],
	[12, 26],
	[13, 23],
	[14, 28],
	[1, 17],
	[11, 11],
	[13, 26]
]
emergency_vehicle_coords = [6, 17]
width = 16
length = 30
precision = 10
angle_list, probability_list = main_function_to_run(traffic, emergency_vehicle_coords, precision)
goal_list = rotate_goal_probabilites(90,1,0.1)
combined_probabilities = []
for i in range(360):
	combined_probabilities.append(probability_list[i]*goal_list[i])
combined_probabilities = final_interpolate(combined_probabilities)
probability_list = combined_probabilities

# for i in range(45):
# 	y = get_prob_from_angle(traffic, emergency_vehicle_coords, i+1)
# 	print(y)

fig, ax = plt.subplots()

colours = get_colours(probability_list)
# polygons = shaded_polygons(sweep_precision, emergency)
polygons = shaded_polygons(1, emergency_vehicle_coords, width, length)
for i in range(len(polygons)):
	poly = mpatches.Polygon(polygons[i], closed=True, alpha=0.2, color=colours[i], ec=None)
	ax.add_patch(poly)

x_to_plot = []
y_to_plot = []
for item in traffic:
	x_to_plot.append(item[0])
	y_to_plot.append(item[1])

ax.scatter(x_to_plot, y_to_plot, color='black', marker='x')
ax.scatter(emergency_vehicle_coords[0], emergency_vehicle_coords[1], color='red', marker='x')
ax.set_xlim(0, width)
ax.set_ylim(0, length)
plt.show()

plt.style.use('ggplot')

sweeped_angles_enumerated = [i for i, _ in enumerate(angle_list)]

plt.bar(sweeped_angles_enumerated, probability_list, color='green')
plt.xlabel("Angle from North Clockwise")
plt.ylabel("Probability to move")
plt.title("Probability to move at a given angle")

plt.xticks(sweeped_angles_enumerated, angle_list)

plt.show()
