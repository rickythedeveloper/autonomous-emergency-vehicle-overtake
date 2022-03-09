from ...algorithms.continuous.PDFs.PDF_V8 import *

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
f_prob = 1
b_prob = 0.1
g_angle = 0
colours_in_infographic = 7
opacity = 0.2
emergency_colour = 'red'
emergency_marker = 'x'
traffic_colour = 'black'
traffic_marker = 'x'

y = get_prob_from_angle(traffic, emergency_vehicle_coords, 30, f_prob, b_prob, g_angle, precision)
print(y)